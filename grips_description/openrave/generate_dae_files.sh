#!/bin/sh
#~ This script generates the .dae and .urdf files for all the robots 
#~ found in the grips_description/robots 

DESCRIPTION_PKG=`rospack find grips_description`
FILES=$(find $DESCRIPTION_PKG/robots -type f -name *.xacro)

for XACRO in $FILES
do
  FILENAME=$(basename "$XACRO")
  FILENAME="${FILENAME%.*}"
  URDF=$DESCRIPTION_PKG/robots/$FILENAME.urdf
  DAE=$DESCRIPTION_PKG/openrave/$FILENAME.dae
  rosrun xacro xacro.py $XACRO > $URDF
  rosrun collada_urdf urdf_to_collada $URDF $DAE
  echo "URDF successfully generated for [$FILENAME.xacro]"
  #~ rosrun moveit_ikfast round_collada_numbers.py $DAE $DAE 5 > /dev/null
done

