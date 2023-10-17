###########################
## Package Configuration ##
###########################

cmake_minimum_required(VERSION 3.0.2)
project(picker_demo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Picker.srv
  Speak.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  picker_demo
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)