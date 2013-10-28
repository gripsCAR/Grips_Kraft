#!/bin/sh
DESCRIPTION_PKG=`rospack find grips_description`
#~ Grips
GRIPS_XACRO=$DESCRIPTION_PKG/robots/grips.xacro
GRIPS_URDF=$DESCRIPTION_PKG/robots/grips.urdf
GRIPS_DAE=$DESCRIPTION_PKG/openrave/grips.dae
rosrun xacro xacro.py $GRIPS_XACRO > $GRIPS_URDF
rosrun collada_urdf urdf_to_collada $GRIPS_URDF $GRIPS_DAE
echo "URDF successfully written to $GRIPS_URDF"
#~ rosrun moveit_ikfast round_collada_numbers.py $GRIPS_DAE $GRIPS_DAE 5 > /dev/null 
#~ Grips with the simple gripper
GRIPS_XACRO=$DESCRIPTION_PKG/robots/grips_simple_gripper.xacro
GRIPS_URDF=$DESCRIPTION_PKG/robots/grips_simple_gripper.urdf
GRIPS_DAE=$DESCRIPTION_PKG/openrave/grips_simple_gripper.dae
rosrun xacro xacro.py $GRIPS_XACRO > $GRIPS_URDF
rosrun collada_urdf urdf_to_collada $GRIPS_URDF $GRIPS_DAE
echo "URDF successfully written to $GRIPS_URDF"
#~ rosrun moveit_ikfast round_collada_numbers.py $GRIPS_DAE $GRIPS_DAE 5 > /dev/null 
