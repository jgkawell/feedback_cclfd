cmake_minimum_required(VERSION 2.6)
project(feedback_cclfd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  visualization_msgs
  geometry_msgs
  sensor_msgs
)

find_package(OpenCV)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Classification.msg
  ConstraintTypes.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestFeedback.srv
  PerformDemonstration.srv
  TTS.srv
  STT.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
