cmake_minimum_required(VERSION 2.8.3)
project(grips_gazebo)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
  gazebo_ros 
	grips_description 
	grips_control
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
