cmake_minimum_required(VERSION 2.8.3)
project(elcash_drone)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
 FILES
 DroneStatus.msg
 DroneLandTakeoff.msg
 DroneCommand.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
