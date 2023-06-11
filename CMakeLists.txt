cmake_minimum_required(VERSION 3.0.0)
project(toy_slam VERSION 0.1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    -Wall
    -std=c++2a
    -Wno-ignored-attributes
    -Wno-int-in-bool-context
)

include_directories(${PROJECT_SOURCE_DIR})
link_libraries(libpthread.so pthread)

find_package(spdlog CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package (GeographicLib CONFIG REQUIRED)

# 传感器部分
file(GLOB_RECURSE sensor_SRCS "toy_slam/sensor/*.cpp")
add_library(sensor_impl STATIC ${sensor_SRCS})
target_include_directories(sensor_impl PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(sensor_impl PUBLIC ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(sensor_impl PUBLIC  ${PCL_LIBRARIES})
target_link_libraries (sensor_impl PUBLIC ${GeographicLib_LIBRARIES})

# 建图部分
file(GLOB_RECURSE mapping_SRCS "toy_slam/mapping/*.cpp")
add_library(mapping_impl STATIC ${mapping_SRCS})
target_link_libraries(mapping_impl PUBLIC sensor_impl)


# ROS 节点
set(CMAKE_PREFIX_PATH /opt/ros/kinetic)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)

add_executable(toy_slam node.cpp)
target_include_directories(toy_slam PRIVATE .)
target_include_directories(toy_slam PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(toy_slam PRIVATE libpthread.so pthread sensor_impl mapping_impl)
target_link_libraries(toy_slam PRIVATE ${catkin_LIBRARIES})
target_link_libraries(toy_slam PRIVATE spdlog::spdlog)

target_compile_options(
    toy_slam
    PRIVATE
    -Og
    -g
    -ggdb
    -std=c++2a
)
