cmake_minimum_required(VERSION 2.8.3)
project(project_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    stop.msg
    depth.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    direction.srv
    global_path.srv
    exploration.srv
    distance.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
