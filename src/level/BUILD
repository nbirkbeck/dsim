
cc_library(
  name = "trip_stats",
  srcs = ["trip_stats.h", "trip_stats.cc"],
)

cc_library(
 name = "level",
 srcs = [
   "intersection_control.h",
   "parking_lot.h",
   "road_segment.h",
   "level.h",
 ],
 deps = [
   "//:level_proto_cc",
   ":trip_stats",
 ],
 visibility = ["//visibility:public"],
)