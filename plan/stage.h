#ifndef DSIM_PLAN_STAGE_H_
#define DSIM_PLAN_STAGE_H_ 1

#include <vector>
#include <glog/logging.h>
#include <nmath/vec2.h>
#include <nmisc/heap.h>

#include "level/road_segment.h"
#include "math.h"

static constexpr double kRoundingDist = 0.9;

namespace plan {

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
      if (segment_index + 1 < (int)segments.size()) {
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

}  // namespace plan

#endif  // DSIM_PLAN_STAGE_H_
