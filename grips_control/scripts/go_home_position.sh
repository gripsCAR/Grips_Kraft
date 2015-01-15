#!/bin/sh
rostopic pub --once /SA/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /SE/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /linkage_tr/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /WP/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /WY/command std_msgs/Float64 0 > /dev/null &
rostopic pub --once /WR/command std_msgs/Float64 0
