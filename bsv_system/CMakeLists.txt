cmake_minimum_required(VERSION 2.8.3)
project(bsv_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  visualization_msgs
  bsv_msg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
