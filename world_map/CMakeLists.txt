cmake_minimum_required(VERSION 2.8.3)
project(world_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(world_map_node src/world_map_node.cpp)
target_link_libraries(world_map_node ${catkin_LIBRARIES})

