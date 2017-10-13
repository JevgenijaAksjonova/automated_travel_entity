cmake_minimum_required(VERSION 2.8.3)
project(uarm_controller)


find_package(catkin REQUIRED COMPONENTS
    roscpp geometry_msgs uarm
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/uarm_control.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
