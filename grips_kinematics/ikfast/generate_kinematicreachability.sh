#!/bin/sh
#~ rosrun grips_description generate_dae_files.sh
#~ Grips
DESCRIPTION_PKG=`rospack find grips_description`
KINEMATICS_PKG=`rospack find grips_kinematics`
GRIPS_DAE=$DESCRIPTION_PKG/openrave/grips.dae
ROBOT_XML=$DESCRIPTION_PKG/openrave/grips.robot.xml
#~ transformation6d
MSG_KIN_REACH="Generating [kinematicreachability] from: $GRIPS_DAE"
echo $MSG_KIN_REACH
openrave.py --database kinematicreachability --robot=$ROBOT_XML --xyzdelta=0.01 --quatdelta=0.1 --numthreads=8
#~ Show
openrave.py --database kinematicreachability --robot=$ROBOT_XML --show
