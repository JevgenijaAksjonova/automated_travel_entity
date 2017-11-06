cmake_minimum_required(VERSION 2.8.3)
project(uarm_cv_integration)


find_package(catkin REQUIRED COMPONENTS
    roscpp geometry_msgs uarm
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/arm_cv_integration.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
