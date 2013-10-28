cmake_minimum_required(VERSION 2.8.3)
project(grips_gazebo)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros)

catkin_package(
  CATKIN_DEPENDS
    grips_description
    grips_control
    roscpp
    gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
