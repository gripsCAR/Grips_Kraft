#!/usr/bin/env python
import rospy, os, collections
import numpy as np
# PyKDL
import PyKDL
from hrl_geom.pose_converter import PoseConv
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_list_to_kdl
# Utils
from baxter_teleop.utils import read_parameter
from grips_teleop.filters import best_fit_foaw, savitzky_golay, smooth_diff
# Messages
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3


class DynamicCompensation(object):
  def __init__(self):
    self.fs = read_parameter('/joint_state_controller/publish_rate', 1000.0)
    self.window_size = 100
    self.kernel_size = 51
    self.order = 4
    self.a_x = collections.deque(maxlen=self.window_size)
    self.a_y = collections.deque(maxlen=self.window_size)
    self.a_z = collections.deque(maxlen=self.window_size)
    self.alpha_x = collections.deque(maxlen=self.window_size)
    self.alpha_y = collections.deque(maxlen=self.window_size)
    self.alpha_z = collections.deque(maxlen=self.window_size)
    # Set-up publishers/subscribers
    self.acc_pub = rospy.Publisher('/grips/endpoint_acc', Twist)
    rospy.Subscriber('/grips/endpoint_state', EndpointState, self.endpoint_cb)
  
  def endpoint_cb(self, msg):
    self.a_x.append(msg.twist.linear.x)
    self.a_y.append(msg.twist.linear.y)
    self.a_z.append(msg.twist.linear.z)
    self.alpha_x.append(msg.twist.angular.x)
    self.alpha_y.append(msg.twist.angular.y)
    self.alpha_z.append(msg.twist.angular.z)
    if len(self.a_x) < self.window_size:
      return
    acc_msg = Twist()
    ax = savitzky_golay(np.array(self.a_x), self.kernel_size, self.order, deriv=1, rate=self.fs)
    ay = savitzky_golay(np.array(self.a_y), self.kernel_size, self.order, deriv=1, rate=self.fs)
    az = savitzky_golay(np.array(self.a_z), self.kernel_size, self.order, deriv=1, rate=self.fs)
    acc_msg.linear = Vector3(ax[-1], ay[-1], az[-1])
    try:
      self.acc_pub.publish(acc_msg)
    except:
      pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  dc = DynamicCompensation()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
