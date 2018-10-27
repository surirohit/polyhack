cmake_minimum_required(VERSION 2.8.3)
project(elcash_drone)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
 FILES
 DroneStatus.msg
)

catkin_package()
