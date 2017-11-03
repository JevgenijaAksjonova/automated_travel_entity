cmake_minimum_required(VERSION 2.8.3)
project(filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  nav_msgs
  phidgets
  std_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(filter_node src/filter_node.cpp src/localization_global_map.cpp include/particle.h include/localization_global_map.h src/measurements.cpp include/measurements.h)
target_link_libraries(filter_node test_node ${catkin_LIBRARIES})
