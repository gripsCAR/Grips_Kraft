cmake_minimum_required(VERSION 2.8.3)
project(grips_bimanual)

find_package(catkin REQUIRED COMPONENTS
  grips_control
  grips_gazebo
  grips_kinematics
  grips_msgs
  message_generation
)

catkin_package(
CATKIN_DEPENDS
  grips_control 
  grips_gazebo 
  grips_kinematics 
  grips_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
