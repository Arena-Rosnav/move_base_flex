cmake_minimum_required(VERSION 3.0.2)
project(inter_util)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pedsim_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inter_util
  CATKIN_DEPENDS roscpp pedsim_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(inter_util src/inter_util.cpp)
target_link_libraries(inter_util ${catkin_LIBRARIES})
