cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

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

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
target_link_libraries(obstacle_detection_node ${catkin_LIBRARIES})
