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
    const dsim::RoadSegment* road_segment = nullptr;
    int start_index = 0;
    int end_index = 0;
    Segment(const dsim::RoadSegment* s = nullptr,
            int si=0, int ei=0): road_segment(s), start_index(si), end_index(ei) {

    }
  };

  Stage(Type t = NONE): type(t) {}

  Type type;
  dsim::Point point;
  std::vector<Segment> segments;  
};

bool IsNear(const double x, const double y) {
  return abs(x - y) < 1e-5;
}

bool IsOnRectangle(const dsim::ParkingLot& parking_lot,
                   const dsim::Point& position) {
  const float x0 = parking_lot.position().x() - parking_lot.width() / 2;
  const float x1 = parking_lot.position().x() + parking_lot.width() / 2;
  const float y0 = parking_lot.position().y() - parking_lot.height() / 2;
  const float y1 = parking_lot.position().y() + parking_lot.height() / 2;
  
  return
    ((IsNear(position.x(), x0) || IsNear(position.x(), x1)) &&
     (y0 <= position.y() && position.y() <= y1)) ||
    ((IsNear(position.y(), y0) || IsNear(position.y(), y1)) &&
     (x0 <= position.x() && position.x() <= x1));
}

bool FindSegmentOnRectangle(const dsim::Level& level,
                            const dsim::ParkingLot& parking_lot,
                            const dsim::RoadSegment** segment,
                            int* index, int direction) {
  for (const auto& road_segment : level.road_segment()) {
    const int i0 = (direction == 0) ? 0 : road_segment.points_size() - 1;
    const int i1 = (direction == 0) ? 1 : road_segment.points_size();
    for (int i = i0; i < i1; ++i) {
      if (IsOnRectangle(parking_lot, road_segment.points(i))) {
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

struct PointEqual {
  bool operator()(const dsim::Point& p1, const dsim::Point& p2) const {
    return p1.x() == p2.x() && p1.y() == p2.y();
  }
};

struct PointHash {
  size_t operator()(const dsim::Point& pos) const {
    return Hash(pos);
  }
};

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

std::vector<Stage>
PlanTravel(const dsim::Level& level,
           const dsim::ParkingLot& src_parking_lot,
           int parking_space,
           const dsim::ParkingLot& dest_parking_lot) {
  // Stage 0: Look for an exit point in the parking lot
  const dsim::RoadSegment* exit_segment = 0;
  int exit_index = -1;
  if (!FindSegmentOnRectangle(level, src_parking_lot, &exit_segment, &exit_index, 0)) {
    LOG(ERROR) << "Unable to find exit road segment";
    //return {};
  }

  // Stage 2: Look for an entry point into the parking lot.
  const dsim::RoadSegment* enter_segment = 0;
  int enter_index = -1;
  if (!FindSegmentOnRectangle(level, dest_parking_lot, &enter_segment, &enter_index, 1)) {
    LOG(ERROR) << "Unable to find enter road segment";
    return {};
  }

  // Build the graph
  std::vector<dsim::Point> pos;
  std::unordered_map<dsim::Point, int, PointHash, PointEqual> ipos;
  for (const auto& road_segment : level.road_segment()) {
    for (int pi = 0; pi < road_segment.points_size(); ++pi) {
      const auto& p = road_segment.points(pi);
      if (!ipos.count(p)) {
        ipos[p] = pos.size();
        pos.push_back(p);
      }
    }
  }
  
  std::vector<std::vector<int> > adj(pos.size());
  std::vector<std::vector<std::pair<const dsim::RoadSegment*, int> > > adj_info(pos.size());
  
  int num_arcs = 0;
  for (const auto& road_segment : level.road_segment()) {
    int last_p = -1;
    for (int pi = 0; pi < road_segment.points_size(); ++pi) {
      const auto& p = road_segment.points(pi);
      int cur_p = ipos[p];
      if (last_p >= 0) {
        adj[last_p].push_back(cur_p);
        adj_info[last_p].push_back(std::pair<const dsim::RoadSegment*, int>(&road_segment, pi - 1));
        num_arcs++;
      }
      last_p = cur_p;
    }
  }
  LOG(INFO) << "Adjacency:" << adj.size() << " " << num_arcs;
  // Start position:
  const int a = ipos[exit_segment->points(exit_index)];
  const int b = ipos[enter_segment->points(enter_index)];

  LOG(INFO) << "Searching for path from:" << a << " " << b;
  std::vector<double> dist(pos.size(), -1);
  std::vector<double> g(pos.size(), -1);
  std::vector<double> h(pos.size(), -1);
  std::vector<int> prev(pos.size(), -1);
  std::vector<std::pair<const dsim::RoadSegment*, int> > prev_info(pos.size(), {nullptr, -1});

  Heap heap(pos.size());
  heap.insert(a, 0.0);
  g[a] = 0;
  h[a] = 0;
  bool found = false;
  while (heap.peek() >= 0) {
    double f;
    int top = heap.remove(&f);
    if (dist[top] >= 0) continue;
    dist[top] = f;
    g[top] = f - h[a];
    if (top == b) {
      found = true;
      break;
    }
    for (int i = 0;  i < (int)adj[top].size(); ++i) {
      const int c = adj[top][i];
      if (dist[c] >= 0) {
        continue;
      }

      const double e = Distance(pos[c], pos[top]);
      const double hval = Distance(pos[c], pos[a]);
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
  const dsim::RoadSegment* segment = nullptr;
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
      start_index = prev_info[index].second; // std::min(start_index, prev_info[index].second);
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
    if (s.road_segment) name = s.road_segment->name();
    std::cout << "segment:" << name << " " << s.start_index << " " << s.end_index << "\n";
  }
  
  std::vector<Stage> stages;
  stages.push_back(Stage(Stage::EXIT_PARKING_LOT));
  stages.back().segments.push_back(Stage::Segment{exit_segment, exit_index, exit_index});

  stages.push_back(Stage(Stage::ROAD_TRAVEL));
  stages.back().segments = segments;
  
  stages.push_back(Stage(Stage::FIND_PARKING_SPOT));
  stages.back().segments.push_back(Stage::Segment{enter_segment, enter_index, enter_index});

  return stages;
}


class LevelWindow: public GLWindow {
public:
  LevelWindow(dsim::Level& level,
              Stage& stage) : level_(level), stage_(stage) {
    cpos.z = 100;
    farPlane = 1000;
    setRefreshRate(60);
  }
  
  void drawScene() {
    for (int i = 0; i < level_.parking_lot_size(); ++i) {
      const auto& lot = level_.parking_lot(i);

      glColor3f(1, 1, 1);
      glPushMatrix();
      glTranslatef(lot.position().x(),  lot.position().y(), 0);
      glBegin(GL_LINE_LOOP);
      glVertex2f(-lot.width() / 2, -lot.height() / 2);
      glVertex2f(-lot.width() / 2,  lot.height() / 2);
      glVertex2f( lot.width() / 2,  lot.height() / 2);
      glVertex2f( lot.width() / 2, -lot.height() / 2);
      glEnd();
      glPopMatrix();
    }

    for (int i = 0; i < level_.road_segment_size(); ++i) {
      const auto& segment = level_.road_segment(i);
      glColor3f(1, 0, 0);
      glBegin(GL_LINE_STRIP);
      for (const auto& co : segment.points()) {
        glVertex2f(co.x(), co.y());
      }
      glEnd();
    }

    glColor3f(0, 1, 0);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex2f(car_.x(), car_.y());
    glEnd();
    
  }

  void refresh() {
    static nacb::Timer timer;
    distance_to_travel += 10 * (double)timer; // 1.0/600.0;
    timer.reset();
    
    if (segment_index_ < (int)stage_.segments.size()) {
      const auto* segment = &stage_.segments[segment_index_];
      bool set = false;
      while (!set &&
             sub_index_ + segment->start_index < segment->end_index &&
             segment_index_ < (int)stage_.segments.size()) {
        int i1 = segment->start_index + sub_index_;
        int i2 = segment->start_index + sub_index_ + 1;
        const auto& p1 = segment->road_segment->points(i1);
        const auto& p2 = segment->road_segment->points(i2);
        
        double d = Distance(p2, p1);
        if (distance_to_travel < d) {
          car_ = Interpolate(p1, p2, distance_to_travel / d);
          set = true;
          //distance_to_travel = 0;
          break;
        } else {
          distance_to_travel -= d;
          car_ = p2;
          sub_index_++;
          if (sub_index_ + segment->start_index == segment->end_index) {
            sub_index_ = 0;
            segment_index_++;
          }
        }
      }        
    } else {
      // Stage is complete.
      auto plan = PlanTravel(level_, level_.parking_lot(1), 0,
                             level_.parking_lot(2));
      stage_ = plan[1];
      segment_index_ = 0;
      sub_index_ = 0;
    }
    
    GLWindow::refresh();
  }

  dsim::Level& level_;

  Stage stage_;
  int segment_index_ = 0;
  int sub_index_ = 0;

  dsim::Point car_;
  double distance_to_travel = 0;
};

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  LOG(INFO) << "Loading filename:" << FLAGS_filename;
  dsim::Level level;
  if (!Proto::ReadProto<dsim::Level>(FLAGS_filename, &level)) {
    LOG(ERROR) << "Unable to load level from:" << FLAGS_filename;
  }

  LOG(INFO) << level.DebugString();

  for (int i = 0; i < level.parking_lot_size(); ++i) {
    for (int j = 0; j < level.parking_lot_size(); ++j) {
      if (i == j) continue;
      auto plan = PlanTravel(level, level.parking_lot(i), 0,
                             level.parking_lot(j));
      std::cout << plan.size();
    }
  }

  auto plan = PlanTravel(level, level.parking_lot(0), 0,
                         level.parking_lot(1));

  LevelWindow level_window(level, plan[1]);

  level_window.loop(1);
  return 0;
}
