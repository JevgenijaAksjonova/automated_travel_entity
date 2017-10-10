cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_node src/navigation_node.cpp include/global_path_planner.h include/map_visualization.h src/global_path_planner.cpp src/map_visualization.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES})
add_dependencies(navigation_node geometry_msgs)

#add_executable(global_path_planner src/global_path_planner.cpp)
