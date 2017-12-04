cmake_minimum_required(VERSION 2.8.3)
project(uarm_controller)


find_package(catkin REQUIRED COMPONENTS
    roscpp geometry_msgs uarm  message_generation geometry_msgs
)

add_service_files(
  FILES
  armPickupService.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node src/uarm_control.cpp)

 #target_link_libraries(${PROJECT_NAME}_node
 #  ${catkin_LIBRARIES}
 #)
