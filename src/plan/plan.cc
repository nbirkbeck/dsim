#include "plan/plan.h"

#include <glog/logging.h>
#include <iostream>
#include <unordered_map>

#include <nmisc/heap.h>

#include "level/parking_lot.h"
#include "level/road_segment.h"
#include "math.h"

namespace plan {
bool FindSegmentOnRectangle(Level& level, const ParkingLot& parking_lot,
                            RoadSegment** segment, int* index, int direction) {
  for (auto& road_segment : level.road_segments) {
    if (road_segment.points.size() == 1)
      continue;
    const int i0 = (direction == 0) ? 0 : road_segment.points.size() - 1;
    const int i1 = (direction == 0) ? 1 : road_segment.points.size();
    for (int i = i0; i < i1; ++i) {
      if (IsOnRectangle(parking_lot.pos, parking_lot.width(),
                        parking_lot.height(), road_segment.points[i])) {
        *segment = &road_segment;
        *index = i;
        return true;
      }
    }
  }
  return false;
}

void Planner::Init(Level& level) {
  for (auto& road_segment : level.road_segments) {
    for (int pi = 0; pi < (int)road_segment.points.size(); ++pi) {
      const auto& p = road_segment.points[pi];
      if (!ipos.count(p)) {
        ipos[p] = pos.size();
        pos.push_back(p);
      }
    }
  }

  adj.resize(pos.size());
  adj_info.resize(pos.size());

  int num_arcs = 0;
  for (auto& road_segment : level.road_segments) {
    int last_p = -1;
    for (int pi = 0; pi < (int)road_segment.points.size(); ++pi) {
      const auto& p = road_segment.points[pi];
      int cur_p = ipos[p];
      if (last_p >= 0) {
        adj[last_p].push_back(cur_p);
        adj_info[last_p].push_back(
            std::pair<RoadSegment*, int>(&road_segment, pi - 1));
        num_arcs++;
      }
      last_p = cur_p;
    }
  }
}
std::vector<Stage::Segment> Planner::Plan(int a, int b) {
  std::vector<double> dist(pos.size(), -1);
  std::vector<double> g(pos.size(), -1);
  std::vector<double> h(pos.size(), -1);
  std::vector<int> prev(pos.size(), -1);
  std::vector<std::pair<RoadSegment*, int>> prev_info(pos.size(),
                                                      {nullptr, -1});

  Heap heap(pos.size());
  heap.insert(a, 0.0);
  g[a] = 0;
  h[a] = 0;
  const double kMaxSpeed = 8;

  while (heap.peek() >= 0) {
    double f;
    int top = heap.remove(&f);
    if (dist[top] >= 0)
      continue;
    dist[top] = f;
    g[top] = f - h[top];
    if (top == b) {
      break;
    }
    for (int i = 0; i < (int)adj[top].size(); ++i) {
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
    } else {
      segment = prev_info[index].first;
      end_index = prev_info[index].second;
      start_index = prev_info[index].second;
    }
    if (index == a)
      break;
    index = prev[index];
    ++k;
  }
  if (segment != nullptr) {
    segments.push_back(Stage::Segment(segment, start_index, end_index + 1));
  }

  std::reverse(segments.begin(), segments.end());
  return segments;
}

// Start a car in each parking lot (it can have a home parking spot)
// Give each car a destination (e.g., another parking lot)
// Have each car navigate to its distination
//   It must find its way out of the parking lot
//   To the other parking lot
//   It can look for a spot in the parking lot once it gets there.
//
// Simplifications: start with no collision detection.

std::vector<Stage> Planner::Plan(const ParkingLot& src_parking_lot,
                                 int parking_space,
                                 const ParkingLot& dest_parking_lot,
                                 int dest_parking_spot) {
  // Stage 0: Look for an exit point in the parking lot
  RoadSegment* exit_segment = 0;
  int exit_index = -1;
  if (!FindSegmentOnRectangle(level, src_parking_lot, &exit_segment,
                              &exit_index, 0)) {
    LOG(ERROR) << "Unable to find exit road segment for "
               << src_parking_lot.name;
    return {};
  }

  // Stage 2: Look for an entry point into the parking lot.
  RoadSegment* enter_segment = 0;
  int enter_index = -1;
  if (!FindSegmentOnRectangle(level, dest_parking_lot, &enter_segment,
                              &enter_index, 1)) {
    LOG(ERROR) << "Unable to find enter road segment";
    return {};
  }

  // Start position:
  const int a = ipos[exit_segment->points[exit_index]];
  const int b = ipos[enter_segment->points[enter_index]];
  auto segments = Plan(a, b);

  if (false) {
    std::cout << "segments:\n";
    for (const auto& s : segments) {
      std::string name;
      if (s.road_segment)
        name = s.road_segment->name;
      std::cout << "segment:" << name << " " << s.start_index << " "
                << s.end_index << "\n";
    }
  }
  std::vector<Stage> stages;
  stages.push_back(Stage(Stage::EXIT_PARKING_LOT));
  stages.back().point =
      (src_parking_lot.pos + src_parking_lot.parking_spots[parking_space].pos);
  stages.back().parking_lot = &src_parking_lot;
  stages.back().segments.push_back(
      Stage::Segment{exit_segment, exit_index, exit_index});

  stages.push_back(Stage(Stage::ROAD_TRAVEL, segments));

  stages.push_back(Stage(Stage::FIND_PARKING_SPOT));
  stages.back().segments.push_back(
      Stage::Segment{enter_segment, enter_index, enter_index});
  stages.back().parking_lot = &dest_parking_lot;
  stages.back().point = (dest_parking_lot.pos +
                         dest_parking_lot.parking_spots[dest_parking_spot].pos);

  return stages;
}

std::vector<Stage> PlanTravel(Level& level, const ParkingLot& src_parking_lot,
                              int parking_space,
                              const ParkingLot& dest_parking_lot,
                              int dest_parking_spot) {
  Planner planner(level);
  return planner.Plan(src_parking_lot, parking_space, dest_parking_lot,
                      dest_parking_spot);
}

} // namespace plan
