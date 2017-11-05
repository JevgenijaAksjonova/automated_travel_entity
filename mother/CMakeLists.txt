cmake_minimum_required(VERSION 2.8.3)
project(mother)

find_package(catkin REQUIRED COMPONENTS
  camera
  message_generation
  geometry_msgs
  keyboard_input
  navigation
  odometry
  robot_motor_controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
  uarm_controller
  world_map
)

find_package(OpenCV 2 REQUIRED)

add_message_files(
   FILES
   RAS_Evidence.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mother
  CATKIN_DEPENDS camera geometry_msgs keyboard_input navigation odometry ras_msgs robot_motor_controller roscpp rospy sensor_msgs std_msgs uarm_controller world_map message_runtime
#  DEPENDS system_lib
)