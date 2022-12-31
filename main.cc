#include "utigl/glwindow.h"

#include <GL/gl.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <vector>
#include <unordered_map>
#include <nmisc/heap.h>
#include <nmisc/timer.h>
#include <nmath/vec3.h>
#include <nmath/vec2.h>
#include "level.pb.h"

DEFINE_string(filename, "", "Path to input filename");

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

class RoadSegment {
public:
  RoadSegment(const dsim::RoadSegment& seg):
    name(seg.name()),
    speed_limit(seg.speed_limit()) {
    for (const auto& p: seg.points()) {
      points.push_back(nacb::Vec2d(p.x(), p.y()));
    }
    if (speed_limit <= 0) speed_limit = 8;
    ResetSpeedEstimate();
  }


  void AddSpeedEstimate( double sp) {
    avg_speed += sp;
    num_avg_speed++;
  }

  void ResetSpeedEstimate() {
    avg_speed = speed_limit;
    num_avg_speed = 1;
  }

  double GetAverageSpeed() const {
    return avg_speed / num_avg_speed;
  }
  
  std::string name;
  double speed_limit;
  std::vector<nacb::Vec2d> points;

  double avg_speed = 0;
  int num_avg_speed = 0;
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

class Level {
public:
  Level(const dsim::Level& level) {
    for (const auto& lot : level.parking_lot()) {
      parking_lots.push_back(ParkingLot(lot));
    }
    for (const auto& seg : level.road_segment()) {
      road_segments.push_back(RoadSegment(seg));
    }
  }
  std::vector<ParkingLot> parking_lots;
  std::vector<RoadSegment> road_segments;
};


// Start a car in each parking lot (it can have a home parking spot)
// Give each car a destination (e.g., another parking lot)
// Have each car navigate to its distination
//   It must find its way out of the parking lot
//   To the other parking lot
//   It can look for a spot in the parking lot once it gets there.
//
// Simplifications: start with no collision detection.

struct Stage {
  enum Type {
     NONE=0,
     EXIT_PARKING_LOT,
     FIND_PARKING_SPOT,
     ROAD_TRAVEL,
  };

  struct Segment {
    RoadSegment* road_segment = nullptr;
    int start_index = 0;
    int end_index = 0;
    Segment(RoadSegment* s = nullptr,
            int si=0, int ei=0): road_segment(s), start_index(si), end_index(ei) {

    }
  };

  Stage(Type t = NONE): type(t) {}

  Type type;
  nacb::Vec2d point;
  std::vector<Segment> segments;
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

dsim::Point Add(const dsim::Point& p1, const dsim::Point p2) {
  dsim::Point p;
  p.set_x(p1.x() + p2.x());
  p.set_y(p1.y() + p2.y());
  return p;
}

dsim::Point Sub(const dsim::Point& p1, const dsim::Point p2) {
  dsim::Point p;
  p.set_x(p1.x() - p2.x());
  p.set_y(p1.y() - p2.y());
  return p;
}

double Distance(const dsim::Point& p1, const dsim::Point p2) {
  const double dx = p1.x() - p2.x();
  const double dy = p1.y() - p2.y();
  return sqrt(dx * dx + dy * dy);
}

dsim::Point Interpolate(const dsim::Point& p1, const dsim::Point p2, double t) {
  dsim::Point p;
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  p.set_x(p1.x() + dx * t);
  p.set_y(p1.y() + dy * t);
  return p;
}

nacb::Vec2d Interpolate(const nacb::Vec2d& p1, const nacb::Vec2d& p2, double t) {
  return p1 * (1.0 - t) + p2 * t;
}

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
      LOG(INFO) << edge_speed;
        
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
      Stage::Segment s;
      s.road_segment = segment;
      s.start_index = start_index;
      s.end_index = end_index + 1;
      segments.push_back(s);

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
    Stage::Segment s;
    s.road_segment = segment;
    s.start_index = start_index;
    s.end_index = end_index + 1;
    segments.push_back(s);
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
                            
  stages.back().segments.push_back(Stage::Segment{exit_segment, exit_index, exit_index});

  stages.push_back(Stage(Stage::ROAD_TRAVEL));
  stages.back().segments = segments;
  
  stages.push_back(Stage(Stage::FIND_PARKING_SPOT));
  stages.back().segments.push_back(Stage::Segment{enter_segment, enter_index, enter_index});
  stages.back().point = (dest_parking_lot.pos +
                         dest_parking_lot.parking_spots[dest_parking_spot].pos);
  
  return stages;
}

class Car {
public:
  Car(Level& level, int id, int start, int end) :
      level_(level), car_id_(id), start_(start), end_(end) {
    plan_ = PlanTravel(level, level.parking_lots[start], id % level.parking_lots[start].parking_spots.size(),
                       level.parking_lots[end], id % level.parking_lots[end].parking_spots.size());
    stage_index_ = 0;
    pos_ = (level.parking_lots[start].parking_spots[0].pos + level.parking_lots[start].pos);
      
    max_speed_ = 6 + 4.0 * rand() / RAND_MAX;
    BuildStaticAccelerationPolicy();
    //wait_ = 1.5 * rand() / RAND_MAX;
  }

  void BuildStaticAccelerationPolicy() {
    // Loop through segments
    double distance = 0;
    std::vector<std::pair<double, int> > accel;
    accel.push_back(std::pair<double, int>(0, 1));
    for (int si = 0; si < (int)plan_[1].segments.size(); ++si) {
      const auto& segment = plan_[1].segments[si];
      double segment_distance = 0;
      for (int i = 0; (segment.start_index + i)  % (int)segment.road_segment->points.size() != segment.end_index; ++i) {
        int i1 = (segment.start_index + i) % (int)segment.road_segment->points.size() ;
        int i2 = (segment.start_index + i + 1) % (int)segment.road_segment->points.size();
        const auto& p1 = segment.road_segment->points[i1];
        const auto& p2 = segment.road_segment->points[i2];
        segment_distance += (p1 - p2).len();
      }
      accel.push_back(std::pair<double, int>(distance + segment_distance - 2, -1));
      accel.push_back(std::pair<double, int>(distance + segment_distance - 0.1, 1));
      distance += segment_distance;
    }
    for (const auto& a : accel) {
      LOG(INFO) << a.first << " " << a.second;
    }
    accel_ = accel;
  }

  void Step(std::vector<Car>& cars,
            double dt) {
    breaking_ = false;
    // Acceleration / deceleration
    // Need to decelerate when approaching a stop (or when getting too close to another)
    if (wait_ > 0) {
      wait_ -= dt;
      if (wait_ < 0) {
        /// Make new plan
        start_ = int(0.99999 * level_.parking_lots.size() * rand()  / RAND_MAX);
        if (start_ == end_) {
          start_ = (start_ + 1) % level_.parking_lots.size();
        }
        plan_ = PlanTravel(level_, level_.parking_lots[end_], car_id_ % 4,
                           level_.parking_lots[start_], car_id_ % 4);
        std::swap(start_, end_);
        LOG(INFO) << "Planning from:" << start_ << " to " << end_;

        stage_index_ = 0;
        segment_index_ = 0;
        sub_index_ = 0;
        distance_travelled_so_far_ = 0;
        distance_to_travel = 0;
        wait_ = 0;
        
        BuildStaticAccelerationPolicy();
      } else {
        return;
      }
    }
    int accel = 1;
    for (int i = 0; i < (int)accel_.size(); ++i) {
      if (distance_travelled_so_far_ >= accel_[i].first &&
          ((i == (int)accel_.size() - 1) ||
           distance_travelled_so_far_ < accel_[i + 1].first)) {
        accel = accel_[i].second;
        break;
      }
    }

    const auto dpos = (pos_ -  last_pos_);
    const double dpos_len = dpos.len();
    for (const auto& car: cars) {
      if (&car == this) continue;
      const auto d_car = (car.pos_ - pos_);
      const double d = d_car.len();

      if (d > 3.5) continue;
      if (d_car.dot(dpos) > 0.71 * d * dpos_len) {
        if (d < 2) {
          accel += -32;
        } else {
          accel += -16;
        }
        breaking_ = true;
      }
    }
    
    if (accel > 0) {
      speed_ += 4.0 * dt;
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
    }

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
        while (!set &&
               (sub_index_ + segment->start_index) % segment->road_segment->points.size() != segment->end_index &&
               segment_index_ < (int)stage.segments.size()) {
          int i1 = (segment->start_index + sub_index_) % segment->road_segment->points.size() ;
          int i2 = (segment->start_index + sub_index_ + 1) % segment->road_segment->points.size();
          const auto& p1 = segment->road_segment->points[i1];
          const auto& p2 = segment->road_segment->points[i2];

          // This is a special hack for a looped segment
          const double d = (p2 - p1).len();
          if (d == 0) {
            sub_index_++;
            continue;
          }
          if (distance_to_travel < d) {
            pos_ = Interpolate(p1, p2, distance_to_travel / d);
            set = true;
            segment->road_segment->AddSpeedEstimate((pos_ - last_pos_).len());
            break;
          } else {
            distance_to_travel -= d;
            pos_ = p2;
            sub_index_++;

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
      }
    }
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
  nacb::Vec2d pos_;
  nacb::Vec2d last_pos_;
  bool breaking_ = false;
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
      for (int i = 0; i < 4 * 7; ++i) {
        cars_.push_back(Car(level, i, (i/4) % 7, ((i/4) + 1) % 7));
      }
      for (int i = 0; i < 16; ++i) {
        cars_.push_back(Car(level, i, 7, ((i/4) + 1) % 7));
      }
    }

  }
  bool keyboard(unsigned char c, int x, int y) {
    if (c == ' ') {
      cars_[0].speed_ = 0;
    }
    return true;
  }
  void drawScene() {
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
    }

    glPointSize(3);
    glBegin(GL_POINTS);
    for (const auto& car: cars_) {
      glColor3f(0, 1, 0);
      if (car.breaking_ && 0) {
        glColor3f(0, 0, 1);
      }

      glVertex2f(car.pos().x, car.pos().y);
    };
    glEnd();
    
  }

  void refresh() {
    static nacb::Timer timer;
    const double dt = (double)timer;
    timer.reset();
    for (auto& car: cars_) {
      car.Step(cars_, dt);
    }
    
    GLWindow::refresh();
  }

  Level& level_;
  std::vector<Car> cars_;
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

  level_window.loop(1);
  return 0;
}
