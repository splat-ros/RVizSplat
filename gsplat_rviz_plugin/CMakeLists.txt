cmake_minimum_required(VERSION 3.10)

project(gsplat_rviz_plugin)

set(QT_NO_CREATE_VERSIONLESS_TARGETS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(gsplat_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDAToolkit QUIET)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 5)
endif()
set(QT_VERSION "${Qt${QT_VERSION_MAJOR}_VERSION}")
if(${QT_VERSION} VERSION_LESS 5.15.0)
  function(qt_wrap_cpp out)
    qt5_wrap_cpp(_sources ${ARGN})
    set("${out}" ${_sources} PARENT_SCOPE)
  endfunction()
endif()

set(gsplat_rviz_plugin_headers_to_moc
  include/gsplat_rviz_plugin/displays/gsplat_display.hpp
)

foreach(header "${gsplat_rviz_plugin_headers_to_moc}")
  qt_wrap_cpp(gsplat_rviz_plugin_moc_files "${header}")
endforeach()

set(gsplat_rviz_plugin_source_files
  src/gsplat_rviz_plugin/displays/gsplat_display.cpp
  src/gsplat_rviz_plugin/splat_cloud.cpp
  src/gsplat_rviz_plugin/perf_monitor.cpp
  src/gsplat_rviz_plugin/sorters/cpu_sorter.cpp
  src/gsplat_rviz_plugin/sorters/sorter_factory.cpp
  src/gsplat_rviz_plugin/splat_loaders/ply_loader.cpp
  src/gsplat_rviz_plugin/splat_loaders/ply_file_source.cpp
  src/gsplat_rviz_plugin/splat_loaders/ros_topic_source.cpp
  src/gsplat_rviz_plugin/splat_loaders/compact_decoder.cpp
  src/gsplat_rviz_plugin/splat_loaders/blob_assembler.cpp
  src/gsplat_rviz_plugin/splat_loaders/snapshot_payload_loader.cpp
  src/gsplat_rviz_plugin/splat_loaders/snapshot_ref_source.cpp
  src/gsplat_rviz_plugin/splat_loaders/blob_topic_source.cpp
  src/gsplat_rviz_plugin/splat_loaders/tile_cache.cpp
  src/gsplat_rviz_plugin/splat_loaders/tile_stream_source.cpp
  src/gsplat_rviz_plugin/transparency/wboit_compositor.cpp
)

if(CUDAToolkit_FOUND)
  message(WARNING "gsplat [INFO]: CUDA toolkit found (version ${CUDAToolkit_VERSION}) — building GPU sort")
  enable_language(CUDA)
  if(NOT CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  endif()
  list(APPEND gsplat_rviz_plugin_source_files
    src/gsplat_rviz_plugin/sorters/cuda_sorter.cu)
else()
  message(WARNING "gsplat [INFO]: CUDA toolkit not found — using CPU pdqsort fallback")
endif()

add_library(gsplat_rviz_plugin SHARED
  ${gsplat_rviz_plugin_moc_files}
  ${gsplat_rviz_plugin_source_files}
)

target_include_directories(gsplat_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(gsplat_rviz_plugin PUBLIC
  Qt${QT_VERSION_MAJOR}::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
  GLEW::GLEW
  Boost::boost
  ${geometry_msgs_TARGETS}
  gsplat_msgs::gsplat_msgs__rosidl_typesupport_cpp
)
#target_link_libraries(gsplat_rviz_plugin PUBLIC "${cpp_typesupport_target}")

target_compile_definitions(gsplat_rviz_plugin PRIVATE "GSPLAT_RVIZ_PLUGIN_BUILDING_LIBRARY")

if(CUDAToolkit_FOUND)
  target_compile_definitions(gsplat_rviz_plugin PRIVATE GSPLAT_USE_CUDA)
  target_link_libraries(gsplat_rviz_plugin PRIVATE CUDA::cudart)
endif()

register_rviz_ogre_media_exports(DIRECTORIES 
  "ogre_media/materials/scripts"
  "ogre_media/materials/glsl120"
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(gsplat_rviz_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  gsplat_msgs
  geometry_msgs
  Qt${QT_VERSION_MAJOR}
)

install(
  TARGETS gsplat_rviz_plugin
  EXPORT gsplat_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
