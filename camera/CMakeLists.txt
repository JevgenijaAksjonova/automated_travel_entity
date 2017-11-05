cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
  std_msgs
  sensor_msgs
  ras_msgs
  pcl_conversions
  pcl_ros
  #opencv2
  #cv_bridge
  #rospy
)
find_package(OpenCV 2 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  PosAndImage.msg
)

add_service_files(
  FILES
  recognizer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  ras_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)
