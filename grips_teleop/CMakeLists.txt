cmake_minimum_required(VERSION 2.8.3)
project(grips_teleop)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  rate_position_controller
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  baxter_core_msgs
  baxter_teleop
  grips_description
  grips_moveit_config
  pykdl_utils
  rate_position_controller
  tf_conversions
  urdf_parser_py
)


include_directories(${catkin_INCLUDE_DIRS})
