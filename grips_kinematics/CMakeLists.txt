cmake_minimum_required(VERSION 2.8.3)

project(grips_kinematics)

# Names
set(KIN_INTERFACE grips_kinematic_interface)
set(LIB_6D_NAME   grips_transform6d_ikfast_plugin)
set(LIB_5D_1_NAME grips_translationdirection5d_1_plugin)
set(LIB_5D_2_NAME grips_translationdirection5d_2_plugin)
set(LIB_ITERATIVE grips_iterative_coupling_plugin)

find_package(catkin REQUIRED COMPONENTS 
  grips_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  tf_conversions
  joint_limits_interface
  #eigen_conversions
)

catkin_package( 
  LIBRARIES 
    ${KIN_INTERFACE}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    grips_description
    grips_msgs
    grips_moveit_config
    moveit_core
    pluginlib
    moveit_ros_planning_interface
    interactive_markers
    roscpp
    tf_conversions
    joint_limits_interface
)

catkin_python_setup()

include_directories(include)

find_package(Eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Libraries
add_library(${KIN_INTERFACE} src/kinematic_interface.cpp)
target_link_libraries(${KIN_INTERFACE} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

# Nodes
add_executable(kinematic_services src/kinematic_services.cpp)
target_link_libraries(kinematic_services ${KIN_INTERFACE} ${catkin_LIBRARIES})
add_dependencies(kinematic_services grips_msgs_gencpp)

# Kinematics Plugins
add_library(${LIB_6D_NAME} src/grips_transform6d_ikfast_plugin.cpp)
target_link_libraries(${LIB_6D_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_library(${LIB_5D_1_NAME} src/grips_translationdirection5d_1_plugin.cpp)
target_link_libraries(${LIB_5D_1_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_library(${LIB_5D_2_NAME} src/grips_translationdirection5d_2_plugin.cpp)
target_link_libraries(${LIB_5D_2_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_library(${LIB_ITERATIVE} src/iterative_coupling_plugin.cpp)
target_link_libraries(${LIB_ITERATIVE} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${LIB_6D_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS ${LIB_5D_1_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS ${LIB_5D_2_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS ${LIB_ITERATIVE} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
  grips_transform6d_ikfast_plugin.xml
  grips_translationdirection5d_1_plugin.xml
  grips_translationdirection5d_2_plugin.xml
  iterative_coupling_plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION})
