cmake_minimum_required(VERSION 2.8.3)
project(grips_manipulation)

find_package(catkin REQUIRED COMPONENTS
  grips_control
  grips_kinematics
  grips_msgs
  message_generation
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

add_library(${PROJECT_NAME}_collision_free src/collision_free_motion.cpp)
target_link_libraries(${PROJECT_NAME}_collision_free ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
LIBRARIES
  ${PROJECT_NAME}_collision_free
CATKIN_DEPENDS
  grips_control 
  grips_kinematics 
  grips_msgs
  message_runtime
INCLUDE_DIRS 
  include
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(collision_free_node src/collision_free_node.cpp)
target_link_libraries(collision_free_node ${PROJECT_NAME}_collision_free ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(collision_free_node grips_msgs_gencpp) # wait for msgs to be built

#~ add_executable(bimanual_planning_scene src/bimanual_planning_scene.cpp)
#~ target_link_libraries(bimanual_planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#~ add_dependencies(bimanual_planning_scene grips_msgs_gencpp) # wait for msgs to be built
