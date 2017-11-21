cmake_minimum_required(VERSION 2.8.3)
project(wall_finder)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  nav_msgs
  phidgets
  std_msgs
  filter
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
   ${filter_INCLUDE_DIRS}
)

add_executable(wall_finder_node src/wall_finder_node.cpp ${filter_INCLUDE_DIRS}/localization_global_map.h  ${filter_INCLUDE_DIRS}/measurements.h)
target_link_libraries(wall_finder_node ${catkin_LIBRARIES})
