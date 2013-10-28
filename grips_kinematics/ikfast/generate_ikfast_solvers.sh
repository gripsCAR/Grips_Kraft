#!/bin/sh
rosrun grips_description generate_dae_files.sh
#~ Grips
DESCRIPTION_PKG=`rospack find grips_description`
KINEMATICS_PKG=`rospack find grips_kinematics`
GRIPS_DAE=$DESCRIPTION_PKG/openrave/grips.dae
ROBOT_XML=$DESCRIPTION_PKG/openrave/grips.robot.xml
#~ transformation6d
MSG_IKFAST_6D="Generating [transformation6d] ikfast plugin from: $GRIPS_DAE"
echo $MSG_IKFAST_6D
CPP_IKFAST=$KINEMATICS_PKG/ikfast/transformation6d.cpp
CPP_SRC=$KINEMATICS_PKG/src/grips_transform6d_ikfast_solver.cpp
#~ python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=$GRIPS_DAE --iktype=transform6d --baselink=1 --eelink=7 --savefile=$CPP_IKFAST
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=arm --precision=5
CPP_GEN=~/.openrave/kinematics.282f072fb772a7cf8cc58947f21dd6b1/ikfast61.Transform6D.0_1_2_3_4_5.cpp
cp $CPP_GEN $CPP_IKFAST
cp $CPP_IKFAST $CPP_SRC
echo "The following files were generated:"
echo $CPP_IKFAST
echo $CPP_SRC
#~ iktest
openrave.py --database inversekinematics --robot=$ROBOT_XML --manipname=arm --usecached --iktests=1000 
