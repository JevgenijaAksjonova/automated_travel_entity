cmake_minimum_required(VERSION 2.8.3)
project(filter)

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

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES})
