cmake_minimum_required(VERSION 2.8.3)
project(rtk_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  )

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime   
)