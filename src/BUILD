DEPS = [
    "@nimage//:lib",
    "@nmath//:lib",
    "@nmisc//:lib",
    "@libglog//:lib",
    "@gflags//:lib",
]

package_group(
    name = "subpackages",
    packages = [
        "//src/...",
    ],
)

proto_library(
  name = "level_proto",
  srcs = ["level.proto"],
)

cc_proto_library(
    name = "level_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":level_proto",
    ],
)

cc_library(
  name = "math",
  srcs = ["math.h"],
  visibility = ["//visibility:public"],
  deps = [":level_proto_cc",]
)

cc_library(
  name = "proto",
  srcs = ["proto.h"],
  deps = [":level_proto_cc",]
)


cc_library(
  name = "car",
  srcs = ["car.h"],
  deps = [
    "//level",
    "//plan"
 ]
)


cc_binary(
  name = "main",
  srcs = ["main.cc"],
  deps = [
     ":car",
     ":math",
     ":proto",
     ":level_proto_cc",
     "//level",
     "//plan",
     "//utigl:glwindow",
     "//utigl:ffont",
  ] + DEPS,
  linkopts = [
     "-lGL -lGLU -lglut -lGLEW"
  ],
  copts = [
    "-mmmx -msse -msse3"
  ]
)