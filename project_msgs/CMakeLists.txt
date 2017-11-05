cmake_minimum_required(VERSION 2.8.3)
project(project_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    stop.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    direction.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
