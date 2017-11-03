cmake_minimum_required(VERSION 2.8.3)
project(emergency_stop)


## Specify additional locations of header files
## Your package locations should be listed before other locations
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/emergency_stop_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

