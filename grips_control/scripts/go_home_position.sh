#!/bin/sh
rostopic pub --once /grips/SA/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /grips/SE/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /grips/linkage_tr/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /grips/WP/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /grips/WY/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /grips/WR/command std_msgs/Float64 0
