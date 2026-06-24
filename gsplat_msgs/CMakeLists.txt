cmake_minimum_required(VERSION 3.8)
project(gsplat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Splat.msg"
  "msg/SplatArray.msg"
  "msg/SplatBlobChunk.msg"
  "msg/SplatSnapshotRef.msg"
  "msg/SplatStreamInfo.msg"
  "msg/SplatTileChunk.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
