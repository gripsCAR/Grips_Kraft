#!/usr/bin/env python
import rospy, os
# Utils
from hrl_geom.pose_converter import PoseConv
from baxter_teleop.utils import read_parameter
# Messages
from geometry_msgs.msg import PoseStamped


class CartersianTrajectory(object):
  def __init__(self):
    # Parse the yaml file to obtain the trajectory
    time_step = read_parameter('~time_step', 0.05)
    points = read_parameter('~points', list())
    frame_id = read_parameter('~frame_id', 'base_link')
    rate = rospy.Rate(1 / time_step)
    # Set-up publishers/subscribers
    cmd_pub = rospy.Publisher('/grips/ik_command', PoseStamped)
    for point in points:
      #~ if rospy.is_shutdown():
        #~ return
      pose = PoseConv.to_pose_stamped_msg(frame_id, point[4:7], point[:4])
      cmd_pub.publish(pose)
      try:
        rate.sleep()
      except:
        break

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  ct = CartersianTrajectory()
  rospy.loginfo('Shuting down [%s] node' % node_name)
