#include <GL/glew.h>
#include <GL/gl.h>

#include "utigl/glwindow.h"
#include "utigl/ffont.h"


#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <vector>
#include <deque>
#include <unordered_map>
#include <nmisc/heap.h>
#include <nmisc/timer.h>
#include <nmath/vec3.h>
#include <nmath/vec2.h>
#include <nimage/image.h>
#include <nappear/mesh.h>
#include "level.pb.h"

DEFINE_string(filename, "", "Path to input filename");
DEFINE_string(model_dir, "", "Path to models");

class Car;

class Proto {
public:
  template <class T>
  static bool ReadProto(const std::string& filename, T* proto) {
    using namespace google::protobuf;
    if (filename.size() == 0) {
      LOG(ERROR) << "Need at least one argument (the input file)";
      return false;
    }
    const int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
      LOG(ERROR) << "Unable to open input file: " << filename;
      return false;
    }
    io::FileInputStream fstream(fd);
    if (!TextFormat::Parse(&fstream, proto)) {
      LOG(ERROR) << "Unable to parse input:" << filename;
      return false;
    }
    return true;
  }
};

nacb::Vec2d Interpolate(const nacb::Vec2d& p1, const nacb::Vec2d& p2, double t) {
  return p1 * (1.0 - t) + p2 * t;
}


class RoadSegment;

class IntersectionControl {
public:
  IntersectionControl(const dsim::IntersectionControl& control) :
    type(control.type()),
    name(control.name()),
    pos(control.position().x(), control.position().y()),
    dir(control.dir().x(), control.dir().y()),
    time(control.time()) {
  }
  bool IsDirectional() const {
    if (type == dsim::IntersectionControl::STOP) return true;
    if (type == dsim::IntersectionControl::YIELD) return true;
    return false;
  }
  bool IsEnforcedDirection(const nacb::Vec2d& test) const {
    return dir.dot(test) >= 0.999 * test.len();
  }
  bool IsStopSign() const {
    return type == dsim::IntersectionControl::STOP;
  }
  bool IsLight() const {
    return type == dsim::IntersectionControl::LIGHTS;
  }
  bool IsGreen(RoadSegment* segment) const {
    for (int j = 0; j < (int)incoming_segments.size(); ++j) {
      if (segment == incoming_segments[j].first) {
        return parity == j;
      }
    }
    return false;
  }
  void Step(double dt) {
    elapsed += dt;
    if (elapsed >= time) {
      parity = (parity + 1) % incoming_segments.size();
      elapsed = 0;
    }
  }

  std::vector<std::pair<RoadSegment*, int>> incoming_segments;
  
  dsim::IntersectionControl::Type type;
  std::string name;
  nacb::Vec2d pos;
  nacb::Vec2d dir;
  double time;
  int parity = false;
  double elapsed = 0;
};

class RoadSegment {
public:
  static constexpr int kMaxDeckSize = 10;
  struct SpeedEstimateEntry {
    int t = 0;
    double avg_speed = 0;
    double num_avg_speed = 0;
    SpeedEstimateEntry(int time = 0): t(time) {}

    bool operator==(const SpeedEstimateEntry& other) const {
      return t == other.t && avg_speed == other.avg_speed;
    }
    bool operator!=(const SpeedEstimateEntry& other) const {
      return !((*this) == (other));
    }
  };
  RoadSegment(const dsim::RoadSegment& seg):
    name(seg.name()),
    speed_limit(seg.speed_limit()) {
    for (const auto& p: seg.points()) {
      points.push_back(nacb::Vec2d(p.x(), p.y()));
    }
    cars.resize(seg.points().size());
    
    if (speed_limit <= 0) speed_limit = 8;
    
    UpdateSpeedEstimate();
  }

  void AddSpeedEstimate(double t, double sp) {
    if (deck.size() > kMaxDeckSize) {
      deck.pop_front();
    }
    if (deck.empty() || int(t) != deck.back().t) {
      deck.push_back(SpeedEstimateEntry(int(t)));
    }
    deck.back().avg_speed += sp;
    deck.back().num_avg_speed++;
  }

  void UpdateSpeedEstimate() {
    double num_avg_speed = 1.0/8.0;
    avg_speed = num_avg_speed * speed_limit;
    for (const auto& entry : deck) {
      avg_speed += entry.avg_speed;
      num_avg_speed += entry.num_avg_speed;
    }
    avg_speed /= num_avg_speed;
  }

  void SetIntersections(std::vector<IntersectionControl>& isects) {
    for (int j = 0; j <(int)isects.size(); ++j) {
      for (int i = 1; i < (int)points.size(); ++i) {
        if ((points[i] - isects[j].pos).len() < 1e-6) {
          const auto& dir = points[i] - points[i - 1];
          if (!isects[j].IsDirectional() ||
              isects[j].IsEnforcedDirection(dir)) {
            intersections[i] = &isects[j];
          }
          isects[j].incoming_segments.push_back(std::pair<RoadSegment*, int>(this, i - 1));
        }
      }
    }
  }

  double GetAverageSpeed() {
    if (deck.empty() || deck.back() != last_updated) {
      UpdateSpeedEstimate();
      if (deck.size()) last_updated = deck.back();
    }
    return avg_speed;
  }
  
  std::string name;
  double speed_limit;
  std::vector<nacb::Vec2d> points;
  std::unordered_map<int, IntersectionControl*> intersections;
  std::vector<std::set<const Car*> > cars;

  SpeedEstimateEntry last_updated;
  std::deque<SpeedEstimateEntry> deck;
  double avg_speed = 0;
};

class ParkingSpot {
public:
  ParkingSpot(double x, double y): pos(x, y) {}
  nacb::Vec2d pos;
};

class ParkingLot {
public:
  ParkingLot(const dsim::ParkingLot& lot):
    name(lot.name()),
    pos(lot.position().x(), lot.position().y()),
    width_(lot.width()),
    height_(lot.height()) {
    for (const auto& spot : lot.parking_spots()) {
      parking_spots.push_back(ParkingSpot(spot.position().x(), spot.position().y()));
    }
  }

  const double width() const { return width_; }
  const double height() const { return height_; }
  
  std::string name;
  nacb::Vec2d pos;
  double width_, height_;
  std::vector<ParkingSpot> parking_spots;
};

class Stats {
public:
  struct Info {
    double total_time = 0;
    int num_observations = 0;
  };

  void AddObservation(const ParkingLot* src,
                      const ParkingLot* dest,
                      double elapsed_time) {
    Info& info = data[src->name][dest->name];
    info.total_time += elapsed_time;
    info.num_observations++;
  }

  double GetAverageTripTime() {
    double average_time = 0;
    int num_pairs = 0;
    for (const auto& src: data) {
      for (const auto& dest: src.second) {
        const double estimate = dest.second.total_time / std::max(1, dest.second.num_observations);
        average_time += estimate;
        num_pairs++;
      }
    }
    return average_time / num_pairs;
  }

  void Print() {
    double average_time = 0;
    int num_pairs = 0;
    std::cout << "------------------------------------------------------------\n";
    for (const auto& src: data) {
      for (const auto& dest: src.second) {
        const double estimate = dest.second.total_time / std::max(1, dest.second.num_observations);
        std::cout << src.first << " " << dest.first << " " << estimate << std::endl;
        average_time += estimate;
        num_pairs++;
      }
    }
    if (num_pairs)
      std::cout << average_time / num_pairs << "(" << num_pairs << ")\n";
  }

  std::unordered_map<std::string, std::unordered_map<std::string, Info> > data;
};

class Level {
public:
  Level(const dsim::Level& level) {
    for (const auto& lot : level.parking_lot()) {
      parking_lots.push_back(ParkingLot(lot));
    }
    for (const auto& seg : level.road_segment()) {
      road_segments.push_back(RoadSegment(seg));
    }
    for (const auto& isect : level.intersection_control()) {
      intersections.push_back(IntersectionControl(isect));
    }
    for (auto& seg : road_segments) {
      seg.SetIntersections(intersections);
    }
  }
  int PickRandomParkingLot() {
    double r = (double)rand() / RAND_MAX;
    int num_spots = 0;
    for (int i = 0; i < (int)parking_lots.size(); ++i) {
      num_spots += parking_lots[i].parking_spots.size();
    }
    double val = 0;
    for (int i = 0; i < (int)parking_lots.size(); ++i) {
      val += (double)parking_lots[i].parking_spots.size() / num_spots;
      if (val > r) return i;
    }
    return (int)parking_lots.size() - 1;
  }
  Stats stats;
  std::vector<ParkingLot> parking_lots;
  std::vector<RoadSegment> road_segments;
  std::vector<IntersectionControl> intersections;
};


// Start a car in each parking lot (it can have a home parking spot)
// Give each car a destination (e.g., another parking lot)
// Have each car navigate to its distination
//   It must find its way out of the parking lot
//   To the other parking lot
//   It can look for a spot in the parking lot once it gets there.
//
// Simplifications: start with no collision detection.

double EstimateCurveLength(const nacb::Vec2d& a,
                           const nacb::Vec2d& b,
                           const nacb::Vec2d& c) {
  nacb::Vec2d prev;
  double len = 0;
  for (int i = 0; i <= 10; ++i) {
    double t = double(i) / 10;
    nacb::Vec2d point = Interpolate(Interpolate(a, b, t), Interpolate(b, c, t), t);
    if (i) {
      len += (point - prev).len();
    }
    prev = point;
  }
  return len;
}

static constexpr double kRoundingDist = 0.9;

struct Stage {
  enum Type {
     NONE=0,
     EXIT_PARKING_LOT,
     FIND_PARKING_SPOT,
     ROAD_TRAVEL,
  };

  struct Segment {
    struct Info {
      bool smooth_start = false;
      bool smooth_end = false;
      nacb::Vec2d modified_start;
      nacb::Vec2d anchors[2];
      double segment_length = 0;
      double curve_length = 0;
    };
    
    RoadSegment* road_segment = nullptr;
    int start_index = 0;
    int end_index = 0;
    std::vector<Info> info;
    
    Segment(RoadSegment* s = nullptr,
            int si=0, int ei=0): road_segment(s), start_index(si), end_index(ei) {
      if (s) {
        const int len = (end_index - start_index + 2 * (int)road_segment->points.size()) % (int)road_segment->points.size();
        info.resize(len + 1);
      }
    }
    double GetTotalLength(int index) const {
      return (info[index].smooth_end ?
              info[index].segment_length + (info[index].curve_length - kRoundingDist) :
              info[index].segment_length);
    }
      
    void GetPoints(int index,
                   nacb::Vec2d* p1,
                   nacb::Vec2d* p2) const {
      const int i1 = (start_index + index) % (int)road_segment->points.size();
      const int i2 = (start_index + index + 1) % (int)road_segment->points.size();
      *p1 = road_segment->points[i1];
      *p2 = road_segment->points[i2];
    }
    const nacb::Vec2d& GetPoint(int index, int* point_index = nullptr) const {
      const int i = (start_index + index) % (int)road_segment->points.size();
      if (point_index) *point_index = i;
      return road_segment->points[i];
    }
    bool IsLastIndex(int sub_index) const {
      return end_index == (start_index + sub_index) % (int)road_segment->points.size();
    }

    bool GetInterpolatedPoint(int index,
                              double* distance_to_travel,
                              nacb::Vec2d* pos) {
      const double total_curve_length = GetTotalLength(index);
      if (*distance_to_travel > total_curve_length) {
        *distance_to_travel -= total_curve_length;
        return false;
      }
      nacb::Vec2d p1, p2;
      GetPoints(index, &p1, &p2);
      
      const double rounding_start = info[index].segment_length - kRoundingDist;
      const double rounding_t = (*distance_to_travel - rounding_start) /
        (info[index].curve_length);
      if (info[index].smooth_start) {
        p1 = info[index].modified_start;
      }
      if (info[index].smooth_end && rounding_t >= 0) {
        const nacb::Vec2d a = Interpolate(info[index].anchors[0], p2,  rounding_t); // (d - rounding_dist + rounding_t * rounding_dist) / d);
        const nacb::Vec2d b = Interpolate(p2, info[index].anchors[1],  rounding_t); // (d - rounding_dist + rounding_t * rounding_dist) / d);
        *pos = Interpolate(a, b, rounding_t);
      } else {
        *pos = Interpolate(p1, p2, *distance_to_travel / info[index].segment_length);
      }
      return true;
    }

  };

  Stage(Type t = NONE, const std::vector<Segment>& segs = {}): type(t), segments(segs) {
    if (type == ROAD_TRAVEL) BuildCurveInfo();
    LOG(INFO) << "segments_size:" << (int)(segments.size()) << " <-";
  }

  void BuildCurveInfo() {
    const double kSmoothThreshold = 0.95;
    bool prev_smooth = false;
    nacb::Vec2d last_end;
    for (int i = 0; i < (int)segments.size(); ++i) {
      for (int j = 0; !segments[i].IsLastIndex(j); ++j) {
        
        nacb::Vec2d p1, p2, d1;
        segments[i].GetPoints(j, &p1, &p2);
        d1 = (p2 - p1);
        double d1_len = d1.normalize();
        segments[i].info[j].segment_length = d1_len;
        if (prev_smooth) {
          segments[i].info[j].segment_length = (p2 - last_end).len();
          segments[i].info[j].smooth_start = prev_smooth;
          segments[i].info[j].modified_start = last_end;
        }
        
        prev_smooth = false;
        Segment* next_segment = nullptr;
        nacb::Vec2d p3;
        if (GetUpcomingPoint(i, j, &next_segment, &p3)) {
          nacb::Vec2d d2 = p3 - p2;
          const double d2_len = d2.normalize();
          if (d2_len >= 1 &&
              d1_len >= 1 &&
              d1.dot(d2) < kSmoothThreshold) {
            prev_smooth = true;
            segments[i].info[j].smooth_end = true;
            segments[i].info[j].anchors[0] = (p2 - d1 * kRoundingDist);
            segments[i].info[j].anchors[1] = (p2 + d2 * kRoundingDist);
            segments[i].info[j].curve_length = EstimateCurveLength(segments[i].info[j].anchors[0],
                                                                   p2,
                                                                   segments[i].info[j].anchors[1]);
            last_end = segments[i].info[j].anchors[1];
          }
        }
      }
    }
  }

  bool GetPreviousPoint(int segment_index, int sub_index, nacb::Vec2d* p0) {
    if (segment_index == 0 && sub_index == 0) return false;
    if (sub_index == 0) {
      const Segment& prev_segment = segments[segment_index - 1];
      *p0 = prev_segment.road_segment->points[prev_segment.end_index - 1];
    } else {
      const Segment& prev_segment = segments[segment_index];
      *p0 = prev_segment.GetPoint(sub_index - 1);
    }
    return true;
  }
    
  bool GetUpcomingPoint(int segment_index, int sub_index, Segment** next_segment, nacb::Vec2d* p3) {
    if (type != ROAD_TRAVEL) return false;
    *next_segment = nullptr;

    Segment& segment = segments[segment_index];
    if (segment.IsLastIndex(sub_index + 1)) {
      if (segment_index + 1 < segments.size()) {
        *next_segment = &segments[segment_index + 1];
        *p3 = (*next_segment)->GetPoint(1);
        // road_segment->points[((*next_segment)->start_index + 1) % (*next_segment)->road_segment->points.size()];
        return true;
      }
    } else {
      *p3 = segment.GetPoint(sub_index + 2);
      return true;
    }
    return false;
  }
  
  Type type;
  nacb::Vec2d point;
  std::vector<Segment> segments;
  const ParkingLot* parking_lot;
};

bool IsNear(const double x, const double y) {
  return abs(x - y) < 1e-5;
}

bool IsOnRectangle(const ParkingLot& parking_lot,
                   const nacb::Vec2d& pos) {
  const float x0 = parking_lot.pos.x - parking_lot.width() / 2;
  const float x1 = parking_lot.pos.x + parking_lot.width() / 2;
  const float y0 = parking_lot.pos.y - parking_lot.height() / 2;
  const float y1 = parking_lot.pos.y + parking_lot.height() / 2;
 
  return
    ((IsNear(pos.x, x0) || IsNear(pos.x, x1)) &&
     (y0 <= pos.y && pos.y <= y1)) ||
    ((IsNear(pos.y, y0) || IsNear(pos.y, y1)) &&
     (x0 <= pos.x && pos.x <= x1));
}

bool FindSegmentOnRectangle(Level& level,
                            const ParkingLot& parking_lot,
                            RoadSegment** segment,
                            int* index, int direction) {
  for (auto& road_segment : level.road_segments) {
    if (road_segment.points.size() == 1) continue;
    const int i0 = (direction == 0) ? 0 : road_segment.points.size() - 1;
    const int i1 = (direction == 0) ? 1 : road_segment.points.size();
    for (int i = i0; i < i1; ++i) {
      if (IsOnRectangle(parking_lot, road_segment.points[i])) {
        *segment = &road_segment;
        *index = i;
        return true;
      }
    }
  }
  return false;
}

uint64_t Hash(const dsim::Point& pos) {
  int32_t x = (int32_t)pos.x();
  int32_t y = (int32_t)pos.y();
  return (((uint64_t)x) << 32) | ((uint64_t)y);
}

uint64_t Hash(const nacb::Vec2d& pos) {
  int32_t x = (int32_t)pos.x;
  int32_t y = (int32_t)pos.y;
  return (((uint64_t)x) << 32) | ((uint64_t)y);
}

struct PointEqual {
  bool operator()(const dsim::Point& p1, const dsim::Point& p2) const {
    return p1.x() == p2.x() && p1.y() == p2.y();
  }
  bool operator()(const nacb::Vec2d& p1, const nacb::Vec2d& p2) const {
    return p1.x == p2.x && p1.y == p2.y;
  }
};

struct PointHash {
  size_t operator()(const dsim::Point& pos) const {
    return Hash(pos);
  }
  size_t operator()(const nacb::Vec2d& pos) const {
    return Hash(pos);
  }
};

std::vector<Stage>
PlanTravel(Level& level,
           const ParkingLot& src_parking_lot,
           int parking_space,
           const ParkingLot& dest_parking_lot,
           int dest_parking_spot) {
  // Stage 0: Look for an exit point in the parking lot
  RoadSegment* exit_segment = 0;
  int exit_index = -1;
  if (!FindSegmentOnRectangle(level, src_parking_lot, &exit_segment, &exit_index, 0)) {
    LOG(ERROR) << "Unable to find exit road segment";
    //return {};
  }

  // Stage 2: Look for an entry point into the parking lot.
  RoadSegment* enter_segment = 0;
  int enter_index = -1;
  if (!FindSegmentOnRectangle(level, dest_parking_lot, &enter_segment, &enter_index, 1)) {
    LOG(ERROR) << "Unable to find enter road segment";
    return {};
  }

  // Build the graph
  std::vector<nacb::Vec2d> pos;
  std::unordered_map<nacb::Vec2d, int, PointHash, PointEqual> ipos;
  for (auto& road_segment : level.road_segments) {
    for (int pi = 0; pi < (int)road_segment.points.size(); ++pi) {
      const auto& p = road_segment.points[pi];
      if (!ipos.count(p)) {
        ipos[p] = pos.size();
        pos.push_back(p);
      }
    }
  }
  
  std::vector<std::vector<int> > adj(pos.size());
  std::vector<std::vector<std::pair<RoadSegment*, int> > > adj_info(pos.size());
  
  int num_arcs = 0;
  for (auto& road_segment : level.road_segments) {
    int last_p = -1;
    for (int pi = 0; pi < (int)road_segment.points.size(); ++pi) {
      const auto& p = road_segment.points[pi];
      int cur_p = ipos[p];
      if (last_p >= 0) {
        adj[last_p].push_back(cur_p);
        adj_info[last_p].push_back(std::pair<RoadSegment*, int>(&road_segment, pi - 1));
        num_arcs++;
      }
      last_p = cur_p;
    }
  }
  LOG(INFO) << "Adjacency:" << adj.size() << " " << num_arcs;
  // Start position:
  const int a = ipos[exit_segment->points[exit_index]];
  const int b = ipos[enter_segment->points[enter_index]];

  LOG(INFO) << "Searching for path from:" << a << " " << b;
  std::vector<double> dist(pos.size(), -1);
  std::vector<double> g(pos.size(), -1);
  std::vector<double> h(pos.size(), -1);
  std::vector<int> prev(pos.size(), -1);
  std::vector<std::pair<RoadSegment*, int> > prev_info(pos.size(), {nullptr, -1});

  Heap heap(pos.size());
  heap.insert(a, 0.0);
  g[a] = 0;
  h[a] = 0;
  bool found = false;
  const double kMaxSpeed = 8;
  
  while (heap.peek() >= 0) {
    double f;
    int top = heap.remove(&f);
    if (dist[top] >= 0) continue;
    dist[top] = f;
    g[top] = f - h[top];
    if (top == b) {
      found = true;
      break;
    }
    for (int i = 0;  i < (int)adj[top].size(); ++i) {
      const int c = adj[top][i];
      if (dist[c] >= 0) {
        continue;
      }
      const auto& ai = adj_info[top][i];
      const double edge_speed = ai.first->GetAverageSpeed();
      if (edge_speed <= 1e-4) {
        LOG(INFO) << "Edge speed is zero";
        
      }
      const double e = (pos[c] - pos[top]).len() / edge_speed;
      const double hval = (pos[c] - pos[a]).len() / kMaxSpeed;
      h[c] = hval;
      f = (g[top] + e) + hval;
      
      if (heap.exists(c)) {
        if (heap.get(c) > f) {
          heap.update(c, f);
          prev_info[c] = adj_info[top][i];
          prev[c] = top;
        }
      } else {
        heap.insert(c, f);
        prev_info[c] = adj_info[top][i];
        prev[c] = top;
      }
    }
  }
  LOG(INFO) << found << " " << dist[b];
  int index = b;
  RoadSegment* segment = nullptr;
  int end_index = -1;
  int start_index = -1;
  std::vector<Stage::Segment> segments;
  int k = 0;
  while (index >= 0 && k < 1000) {
    if (prev_info[index].first != segment && segment) {
      segments.push_back(Stage::Segment(segment, start_index, end_index + 1));

      segment = prev_info[index].first;
      end_index = prev_info[index].second;
      start_index = prev_info[index].second;
    } else if (segment) {
      start_index = prev_info[index].second;
      // std::min(start_index, prev_info[index].second);
      //end_index = std::max(end_index, prev_info[index].second);
    } else {
      segment = prev_info[index].first;
      end_index = prev_info[index].second;
      start_index = prev_info[index].second;
    }
    if (index == a) break;
    index = prev[index];
    ++k;
  }
  if (segment != nullptr) {
    segments.push_back(Stage::Segment(segment, start_index, end_index + 1));
  }

  std::reverse(segments.begin(), segments.end());

  std::cout << "segments:\n";
  for (const auto& s: segments) {
    std::string name;
    if (s.road_segment) name = s.road_segment->name;
    std::cout << "segment:" << name << " " << s.start_index << " " << s.end_index << "\n";
  }
  
  std::vector<Stage> stages;
  stages.push_back(Stage(Stage::EXIT_PARKING_LOT));
  stages.back().point = (src_parking_lot.pos + 
                         src_parking_lot.parking_spots[parking_space].pos);
  stages.back().parking_lot = &src_parking_lot;                            
  stages.back().segments.push_back(Stage::Segment{exit_segment, exit_index, exit_index});

  stages.push_back(Stage(Stage::ROAD_TRAVEL, segments));
  
  stages.push_back(Stage(Stage::FIND_PARKING_SPOT));
  stages.back().segments.push_back(Stage::Segment{enter_segment, enter_index, enter_index});
  stages.back().parking_lot = &dest_parking_lot;
  stages.back().point = (dest_parking_lot.pos +
                         dest_parking_lot.parking_spots[dest_parking_spot].pos);
  
  return stages;
}

class Car {
public:
  Car(Level& level, int id, int start, int end) :
    level_(level), car_id_(id), start_(start), end_(end), travel_time_(0) {
    plan_ = PlanTravel(level, level.parking_lots[start], id % level.parking_lots[start].parking_spots.size(),
                       level.parking_lots[end], id % level.parking_lots[end].parking_spots.size());
    stage_index_ = 0;
    pos_ = (level.parking_lots[start].parking_spots[0].pos + level.parking_lots[start].pos);
      
    max_speed_ = 12 + 4.0 * rand() / RAND_MAX;
    //wait_ = 1.5 * rand() / RAND_MAX;
  }

  RoadSegment* road_segment() const {
    if (stage_index_ == 1 && stage_index_ < (int)plan_.size()) {
      if (segment_index_ < plan_[stage_index_].segments.size()) {
        return plan_[stage_index_].segments[segment_index_].road_segment;
      }
    }
    return nullptr;
  }

  void BuildNewPlan() {
    /// Make new plan
    start_ = level_.PickRandomParkingLot();
    while (start_ == end_) {
      start_ = level_.PickRandomParkingLot();
    }
    plan_ = PlanTravel(level_, level_.parking_lots[end_], car_id_ % level_.parking_lots[end_].parking_spots.size(),
                       level_.parking_lots[start_], car_id_ % level_.parking_lots[start_].parking_spots.size());
    std::swap(start_, end_);
    LOG(INFO) << "Planning from:" << start_ << " to " << end_;
        
    stage_index_ = 0;
    segment_index_ = 0;
    sub_index_ = 0;
    distance_travelled_so_far_ = 0;
    distance_to_travel = 0;
    wait_ = 0;
    upcoming_intersection_ = nullptr;
  }
  
  void Step(std::vector<Car>& cars,
            double t_abs,
            double dt) {
    breaking_--;
    if (breaking_ < 0) breaking_ = 0;
    if (wait_ > 0) {
      wait_ -= dt;
      if (wait_ < 0) {
        BuildNewPlan();
      } else {
        return;
      }
    }
    // Acceleration / deceleration
    // Need to decelerate when approaching a stop (or when getting too close to another)
    double accel = 1;
    RoadSegment* rs = road_segment();
    if (rs) {
      Stage& stage = plan_[stage_index_];      
      auto* segment = &stage.segments[segment_index_];
      nacb::Vec2d p1, p2, p3;
      segment->GetPoints(sub_index_, &p1, &p2);
      
      Stage::Segment* next_segment = nullptr;
      double max_upcoming_speed = rs->speed_limit;
      if (stage.GetUpcomingPoint(segment_index_, sub_index_, &next_segment, &p3)) {
        if (next_segment) {
          max_upcoming_speed = next_segment->road_segment->speed_limit;
        }
        nacb::Vec2d d1 = (p2 - p1);
        nacb::Vec2d d2 = (p3 - p2);
        d1.normalize();
        d2.normalize();

        if (d1.dot(d2) <= 1e-4) {
          const double kMaxCornerSpeed = 2.5;
          max_upcoming_speed = std::min(max_upcoming_speed, kMaxCornerSpeed);
        }
        double speed_to_shed = speed_ - max_upcoming_speed;
        if (speed_to_shed > 0) {
          const double distance_to_corner = (p2 - pos_).len();
          const double time_to_corner = distance_to_corner / std::max(0.1, speed_);
          const double deceleration_rate = 8;
          const double time_to_shed_speed = speed_to_shed / deceleration_rate;
          if (time_to_corner <= time_to_shed_speed) {
            accel = -1;
            if (time_to_corner < 0.5 * time_to_shed_speed) {
              accel = -4;
            }
            if (time_to_corner < 0.75 * time_to_shed_speed) {
              accel = -2;
            }
          }
        }
      }
    }

    // Intersecton handling
    if (upcoming_intersection_) {
      const auto d_pos = (upcoming_intersection_->pos - pos_);
      const double d = d_pos.len();
      const bool should_slow_down =
        upcoming_intersection_->IsStopSign() ||
        (upcoming_intersection_->IsLight() &&
         !upcoming_intersection_->IsGreen(road_segment()));

      if (should_slow_down) {
        if (d < 0.5) accel = -8;
        else if (d < 1) accel = -4;
        else if (d < 2) accel = -2;
        if (speed_ <= 1 && d >= 1.0) {
          accel = 1.0;
        }
      }
      if (upcoming_intersection_->IsStopSign()) {
        // If we've stopped and we're close enough. If there are no cars, then go.
        if (speed_ <= 0.01 && d <= 1.5) {
          auto* segment = &plan_[stage_index_].segments[segment_index_];
          const RoadSegment* my_road_segment = segment->road_segment;
          bool has_car = false;
          for (const auto& incoming_segs : upcoming_intersection_->incoming_segments) {
            if (incoming_segs.first == my_road_segment) continue;
            if (incoming_segs.first->cars[incoming_segs.second].size()) has_car = true;
          }
          if (!has_car) {
            ignore_intersections_.insert(upcoming_intersection_);
          }
        }
      }
    }

    // Primitive collision avoidance
    const auto dpos = (pos_ -  last_pos_);
    const double dpos_len = dpos.len();
    if (dpos_len > 0) {
      double angle_before = angle_;
      angle_ = atan2(sin(angle_) * 0.4 + 0.6 * dpos.y / dpos_len,
                     cos(angle_) * 0.4 + 0.6 * dpos.x / dpos_len);
      wheel_rot_ = 4 * atan2(sin(angle_ - angle_before),
                             cos(angle_ - angle_before));
    }
    for (const auto& car: cars) {
      if (&car == this) continue;
      if (//!upcoming_intersection_ &&
          car.upcoming_intersection_ &&
          road_segment() != car.road_segment()) continue;
      const auto d_car = (car.pos_ - pos_);
      const double d = d_car.len();

      if (d / std::min(std::max(speed_, 1.0), 4.0) > 1.0) continue;
      if (d_car.dot(dpos) > 0.70 * d * dpos_len) {
        if (d < 1) {
          accel += -16;
        } else if (d < 2) {
          accel += -8;
        } else {
          accel += -2;
        }
      }
    }
    const RoadSegment* my_road_segment = road_segment();
    double speed_limit = my_road_segment ? my_road_segment->speed_limit : 2;
    if (speed_ > speed_limit) {
      accel = -2; // speed_ = my_road_segment->speed_limit;
    }

    if (accel > 0) {
      speed_ += accel * 4.0 * dt;
      if (speed_ >= max_speed_) {
        speed_ = max_speed_;
      }
    } else {
      speed_ += accel * 8.0 * dt;
      if (accel == -1) {
        if (speed_ <= 1.0) {
          speed_ = 1.0;
        }
      } else {
        if (speed_ < 0) speed_ = 0;
      }
      breaking_ = std::min(breaking_ + 1, 4);
    }

    travel_time_ += dt;
    distance_travelled_so_far_ += speed_ * dt;
    distance_to_travel += speed_ * dt;
    last_pos_ = pos_;

    if (plan_[stage_index_].type == Stage::EXIT_PARKING_LOT) {
      const auto* segment = &plan_[stage_index_].segments[0];
      const auto& p1 = segment->road_segment->points[segment->start_index];
      const double d = (plan_[stage_index_].point - p1).len();
      pos_ = Interpolate(plan_[stage_index_].point, p1, std::min(d, distance_to_travel) / d);
      if (distance_to_travel >= d) {
        distance_travelled_so_far_ = 0;
        distance_to_travel = 0;
        stage_index_++;
      }
    } else if (plan_[stage_index_].type == Stage::FIND_PARKING_SPOT) {
      const auto* segment = &plan_[stage_index_].segments[0];
      const auto& p1 = segment->road_segment->points[segment->start_index];
      const double d = (p1 - plan_[stage_index_].point).len();
      pos_ = Interpolate(p1, plan_[stage_index_].point, std::min(d, distance_to_travel) / d);
      if (distance_to_travel >= d) {
        /// Now we've completed the journey
        level_.stats.AddObservation(plan_[0].parking_lot,
                                    plan_[2].parking_lot,
                                    travel_time_);
        travel_time_ = 0;
        distance_travelled_so_far_ = 0;
        distance_to_travel = 0;
        stage_index_ = 0;
        speed_ = 0;
        wait_ = 2.0 * rand() / RAND_MAX + 0.00001;
      }
    } else {
      Stage& stage = plan_[stage_index_];
      if (segment_index_ < (int)stage.segments.size()) {
        auto* segment = &stage.segments[segment_index_];
        bool set = false;

        upcoming_intersection_ = nullptr;
        segment->road_segment->cars[(segment->start_index + sub_index_) % segment->road_segment->points.size()].erase(this);
        
        while (!set &&
               (sub_index_ + segment->start_index) % segment->road_segment->points.size() != segment->end_index &&
               segment_index_ < (int)stage.segments.size()) {
          int i1, i2;
          segment->GetPoint(sub_index_, &i1);
          nacb::Vec2d p2 = segment->GetPoint(sub_index_ + 1, &i2);
          if (segment->road_segment->intersections[i2]) {
            if (!ignore_intersections_.count(segment->road_segment->intersections[i2])) {
              upcoming_intersection_ = segment->road_segment->intersections[i2];
            }
          }
          if (segment->GetInterpolatedPoint(sub_index_, &distance_to_travel, &pos_)) {
            segment->road_segment->cars[i1].insert(this);
            segment->road_segment->AddSpeedEstimate(t_abs, (pos_ - last_pos_).len() / dt);
            set = true;
            break;
          } else {
            const bool zero_length =  (segment->info[sub_index_].segment_length == 0);
            pos_ = p2;
            ignore_intersections_.clear();
            sub_index_++;

            // This is a special hack for a looped segment
            if (zero_length) {
              continue;
            }

            if ((sub_index_ + segment->start_index) % segment->road_segment->points.size() == segment->end_index) {
              sub_index_ = 0;
              segment_index_++;

              if (segment_index_ < (int)stage.segments.size()) {
                segment = &stage.segments[segment_index_];
              } else {
                // Advance stage
                stage_index_++;
                segment = nullptr;
                break;
              }
            }
          }
        }
      } else {
        // What here.
      }
    }
    wheel_anim_ += speed_ / (2 * M_PI * 0.39 / 4.0) * dt;
  }

  const nacb::Vec2d& pos() const {
    return pos_;
  }

  Level& level_;

  std::vector<Stage> plan_;
  int stage_index_ = 0;
  
  std::vector<std::pair<double, int> > accel_;
  
  int car_id_ = 0;
  int segment_index_ = 0;
  int sub_index_ = 0;
  double distance_to_travel = 0;
  double distance_travelled_so_far_ = 0;
  
  double speed_;
  double max_speed_;
  double wait_ = 0;
  
  int start_, end_;
  double travel_time_ = 0;

  nacb::Vec2d pos_;
  nacb::Vec2d last_pos_;
  float angle_ = 0;
  float wheel_rot_ = 0;
  float wheel_anim_ = 0;
  
  int breaking_ = 0;
  IntersectionControl* upcoming_intersection_ = nullptr;
  std::set<IntersectionControl*> ignore_intersections_;
};

class LevelWindow: public GLWindow {
public:
  LevelWindow(Level& level) : level_(level)  {
    cpos.z = 100;
    farPlane = 1000;
    setRefreshRate(60);

    if (false) {
      for (int i = 0; i < 7; ++i) {
        cars_.push_back(Car(level, i, (i) % 7, ((i) + 1) % 7));
      }
    } else {
      for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
        for (int j = 0; j < (int)level.parking_lots[i].parking_spots.size(); ++j) {
          cars_.push_back(Car(level, cars_.size(), i, (i + 1 + rand()) % level.parking_lots.size()));
        }
      }
    }

    ffont_ = FFont("/usr/share/fonts/bitstream-vera/Vera.ttf", 12);
    ffont_.setScale(0.5, 0.5);
    car_mesh_.readObj((FLAGS_model_dir + "/car.obj").c_str());
    car_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    car_mesh_.initNormals(true);

    wheel_mesh_.readObj((FLAGS_model_dir + "/wheel.obj").c_str());
    wheel_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    wheel_mesh_.initNormals(true);

    drop_shadow_mesh_.readObj((FLAGS_model_dir + "/drop_shadow.obj").c_str());
    drop_shadow_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    drop_shadow_mesh_.initNormals(true);

    stop_sign_mesh_.readObj((FLAGS_model_dir + "/stop_sign.obj").c_str());
    stop_sign_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    stop_sign_mesh_.initNormals(true);

    lights_mesh_.readObj((FLAGS_model_dir + "/lights.obj").c_str());
    lights_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    lights_mesh_.initNormals(true);

    light_mesh_.readObj((FLAGS_model_dir + "/light.obj").c_str());
    light_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    light_mesh_.initNormals(true);

    tail_light_mesh_.readObj((FLAGS_model_dir + "/tail_light.obj").c_str());
    tail_light_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    tail_light_mesh_.initNormals(true);
    
    glGenTextures(1, &tex_);
    nacb::Image8 image;
    image.read((FLAGS_model_dir + "/car.png").c_str());
    glBindTexture(GL_TEXTURE_2D, tex_);
    image.initTexture();
  }

  bool keyboard(unsigned char c, int x, int y) {
    if (c == ' ') {
      cars_[0].speed_ = 0;
    }
    if (c == 'f') {
      follow_ = !follow_;
    }
    return true;
  }

  void drawScene() {
    static int count = 0;
    if (count % 100 == 0) {
      level_.stats.Print();
    }
    ++count;

    if (follow_) {
      auto& car = cars_[0];
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      double r = car.angle_;

      glRotatef(15, 1, 0, 0);
      glTranslatef(0, -1.0, -1.0);
      glRotatef(90, -1, 0, 0);
      glRotatef(-180 * r / M_PI + 90, 0, 0, 1);
      glTranslatef(-car.pos().x, -car.pos().y, 0);
    }

    glClearColor(0.5, 0.5, 0.7, 1.0);
    glColor3f(0.05, 0.1, 0.05);
    glDepthMask(0);
    glBegin(GL_QUADS);
    glVertex2f(-100, -100);
    glVertex2f( 100, -100);
    glVertex2f( 100,  100);
    glVertex2f(-100,  100);
    glEnd();
    glDepthMask(1);

    for (int i = 0; i < (int)level_.intersections.size(); ++i) {
      glPushMatrix();
      glTranslatef(level_.intersections[i].pos.x,
                   level_.intersections[i].pos.y,
                   0);
      if (level_.intersections[i].IsStopSign()) {
        glColor3f(1, 0.25, 0.25);
        const double r = atan2(level_.intersections[i].dir.y,
                               level_.intersections[i].dir.x);
        glRotatef(180 * r / M_PI, 0, 0, 1);
        glTranslatef(-0.5, -0.5, 0);
        stop_sign_mesh_.draw();
      } else if (level_.intersections[i].IsLight()) {
        glColor3f(0.5, 0.5, 0.5);
        lights_mesh_.draw();

        for (const auto& segment : level_.intersections[i].incoming_segments) {
          const auto& points = segment.first->points;
          const nacb::Vec2d& p1 = points[segment.second];
          const nacb::Vec2d& p2 = points[(segment.second + 1) % points.size()];
          const nacb::Vec2d dir = p2 - p1;
          const double r = atan2(dir.y, dir.x);
          glPushMatrix();
          glRotatef(180 * r / M_PI, 0, 0, 1);
          if (level_.intersections[i].IsGreen(segment.first)) {
            glColor3f(0.1, 1, 0.1);
          } else {
            glColor3f(1, 0.1, 0.1);
          }
          light_mesh_.draw();
          glPopMatrix();
        }
      }
      glPopMatrix();
    }
    
    for (int i = 0; i < (int)level_.parking_lots.size(); ++i) {
      const auto& lot = level_.parking_lots[i];

      glColor3f(1, 1, 1);
      glPushMatrix();
      glTranslatef(lot.pos.x,  lot.pos.y, 0);
      glBegin(GL_LINE_LOOP);
      glVertex2f(-lot.width() / 2, -lot.height() / 2);
      glVertex2f(-lot.width() / 2,  lot.height() / 2);
      glVertex2f( lot.width() / 2,  lot.height() / 2);
      glVertex2f( lot.width() / 2, -lot.height() / 2);
      glEnd();

      glPointSize(1);
      glBegin(GL_POINTS);
      for (int k = 0; k < (int)lot.parking_spots.size(); ++k) {
        const auto& p = lot.parking_spots[k].pos;
        glVertex2f(p.x, p.y);
      }
      glEnd();
      glPopMatrix();
    }

    for (int i = 0; i < (int)level_.road_segments.size(); ++i) {
      const auto& segment = level_.road_segments[i];

      glColor3f(1, 0, 0);
      glBegin(GL_LINE_STRIP);
      for (const auto& co : segment.points) {
        glVertex2f(co.x, co.y);
      }
      glEnd();

      glColor3f(1, 1, 1);
      glEnable(GL_TEXTURE_2D);
      const double speed = level_.road_segments[i].GetAverageSpeed();
      const nacb::Vec2d co = (segment.points[0] + segment.points[1]) * 0.5;
      char str[1024];
      snprintf(str, sizeof(str), "%3.2f", speed);
      ffont_.drawString(str, co.x, co.y - 0.1);
      glDisable(GL_TEXTURE_2D);
      
    }

    if (car_mesh_.vert.empty()) {
      glPointSize(3);
      glBegin(GL_POINTS);
      for (const auto& car: cars_) {
        glColor3f(0, 1, 0);
        if (car.breaking_ > 1) {
          glColor3f(0, 0, 1);
        }
        glVertex2f(car.pos().x, car.pos().y);
      }
      glEnd();
    }
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHT0);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    
    float pos[4] = {0, 0, 1, 0};
    float white[4] = {1, 1, 1, 1};
    float black[4] = {0, 0, 0, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_AMBIENT, black);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, white);
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      LOG(INFO) << err;
    }
    for (const auto& car: cars_) {
      glColor3f(0.5, 0.5, 0.5);
      glPushMatrix();
      double r = car.angle_;

      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, tex_);

      glColor3f(0.5 + (car.car_id_ % 32) / 64.0,
                0.75 - (car.car_id_ % 32) / 64.0,
                0.25 + (car.car_id_ % 33) / 64.0);
      glTranslatef(car.pos().x, car.pos().y, 0);
      glRotatef(180 * r / M_PI, 0, 0, 1);
      car_mesh_.draw();

      glDisable(GL_TEXTURE_2D);

      if (car.breaking_ >= 1) {
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 0.0, 0.0);
        tail_light_mesh_.draw();
        glEnable(GL_LIGHTING);
      }
      
      glColor3f(0.2, 0.2, 0.2);
      glPushMatrix();
      glTranslatef(0.85/4, -1.0/4.0, 0.4/4);
      r = 0.39 / 4.0 * 2 * M_PI;
      glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
      glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
      wheel_mesh_.draw();
      glPopMatrix();

      glPushMatrix();
      glTranslatef(0.85/4,  1.0/4.0, 0.4/4);
      glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
      glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
      wheel_mesh_.draw();
      glPopMatrix();

      glPushMatrix();
      glTranslatef(-0.85/4,  -1.0/4.0, 0.4/4);
      glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
      wheel_mesh_.draw();
      glPopMatrix();

      glPushMatrix();
      glTranslatef(-0.85/4,  1.0/4.0, 0.4/4);
      glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
      wheel_mesh_.draw();
      glPopMatrix();

      glEnable(GL_BLEND);
      glEnable(GL_TEXTURE_2D);
      glDisable(GL_LIGHTING);
      glColor3f(1, 1, 1);
      drop_shadow_mesh_.draw();
      glEnable(GL_LIGHTING);

      glPopMatrix();
    }
    glDisable(GL_LIGHTING);

    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    double average_trip_time = level_.stats.GetAverageTripTime();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, 0, -20);
    char str[1024];
    snprintf(str, sizeof(str), "ATT: %f", average_trip_time);
    ffont_.drawString(str, -8.f, -6.f);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
  }

  void refresh() {
    static nacb::Timer timer;
    const double dt = (double)timer;
    timer.reset();
    if (dt > 0) {
      for (auto& car: cars_) {
        car.Step(cars_, t_, dt);
      }
      for (auto& isect: level_.intersections) {
        isect.Step(dt);
      }
      t_ += dt;
    }
    GLWindow::refresh();
  }
  double t_ = 0;
  Level& level_;
  std::vector<Car> cars_;
  FFont ffont_;
  bool follow_ = false;
  nappear::Mesh car_mesh_;
  nappear::Mesh wheel_mesh_;
  nappear::Mesh drop_shadow_mesh_;
  nappear::Mesh stop_sign_mesh_;
  nappear::Mesh lights_mesh_;
  nappear::Mesh light_mesh_;
  nappear::Mesh tail_light_mesh_;
  GLuint tex_ = 0;
};

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  LOG(INFO) << "Loading filename:" << FLAGS_filename;
  dsim::Level level_data;
  if (!Proto::ReadProto<dsim::Level>(FLAGS_filename, &level_data)) {
    LOG(ERROR) << "Unable to load level from:" << FLAGS_filename;
  }

  LOG(INFO) << level_data.DebugString();

  Level level(level_data);
  for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
    for (int j = 0; j < (int)level.parking_lots.size(); ++j) {
      if (i == j) continue;
      auto plan = PlanTravel(level, level.parking_lots[i], 0,
                             level.parking_lots[j], 0);
      std::cout << plan.size();
    }
  }

  LevelWindow level_window(level);
  glewInit();
  
  level_window.loop(1);
  return 0;
}
