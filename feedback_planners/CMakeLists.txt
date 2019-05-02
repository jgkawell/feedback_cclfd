cmake_minimum_required(VERSION 2.8.3)
project(feedback_planners)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ConstraintTypes.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestFeedback.srv
  PerformDemonstration.srv
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

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
