cmake_minimum_required(VERSION 2.8.3)
project(grips_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  force_torque_sensor_calib
  gravity_compensation
  netft
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
