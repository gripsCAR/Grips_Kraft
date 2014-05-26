cmake_minimum_required(VERSION 2.8.3)
project(grips_msgs)

find_package(catkin REQUIRED COMPONENTS 
  dynamic_reconfigure
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(FILES 
  GripsState.msg
)

add_service_files(FILES 
  GetPoseMetrics.srv
  GetStateMetrics.srv
  GetJointLimits.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

generate_dynamic_reconfigure_options(
  cfg/KinematicsSolvers.cfg
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  sensor_msgs 
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
