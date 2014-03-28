cmake_minimum_required(VERSION 2.8.3)
project(grips_control)

find_package(catkin REQUIRED COMPONENTS 
  grips_kinematics
  grips_msgs
  grips_moveit_config
  control_msgs
  controller_manager
  eigen_conversions
  message_generation
)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS
  grips_description
  grips_kinematics
  grips_msgs
  grips_moveit_config
  eigen_conversions
  message_runtime
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(HDF5 REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${HDF5_INCLUDE_DIR})

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})
add_dependencies(motion_control grips_msgs_gencpp)

add_executable(build_metrics_index src/build_metrics_index.cpp)
target_link_libraries(build_metrics_index ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})
add_dependencies(build_metrics_index grips_msgs_gencpp)
