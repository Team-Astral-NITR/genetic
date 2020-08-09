cmake_minimum_required(VERSION 2.8.3)
project(genetic_node)
find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  rospy
  std_msgs
  vectornav
  picker_robot
  pid_controller
)
################################################
## Declare ROS messages, services and actions ##
################################################
#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
