cmake_minimum_required(VERSION 2.8.3)
project(grips_worlds)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

catkin_package(
CATKIN_DEPENDS 
  roscpp
  gazebo_ros 
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(dexterity_test_plugin src/dexterity_test_plugin.cpp)
target_link_libraries(dexterity_test_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
