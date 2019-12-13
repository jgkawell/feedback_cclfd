cmake_minimum_required(VERSION 2.6)
project(feedback_visualizations)

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
  message_generation 
  visualization_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
# 	DEPENDENCIES 
# 	std_msgs
# )

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############