#!/usr/bin/env python
import rospy, os, collections
import numpy as np
# Filters
from scipy.signal import lfilter
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
    self.window_size = int(self.fs / 2.0)
    order = int(10000 / self.fs)
    self.b = smooth_diff(order)
    self.vx = collections.deque(maxlen=self.window_size)
    self.vy = collections.deque(maxlen=self.window_size)
    self.vz = collections.deque(maxlen=self.window_size)
    self.wx = collections.deque(maxlen=self.window_size)
    self.wy = collections.deque(maxlen=self.window_size)
    self.wz = collections.deque(maxlen=self.window_size)
    # Set-up publishers/subscribers
    self.acc_pub = rospy.Publisher('/grips/endpoint_acc', Twist)
    rospy.Subscriber('/grips/endpoint_state', EndpointState, self.endpoint_cb)
  
  def endpoint_cb(self, msg):
    self.vx.append(msg.twist.linear.x)
    self.vy.append(msg.twist.linear.y)
    self.vz.append(msg.twist.linear.z)
    self.wx.append(msg.twist.angular.x)
    self.wy.append(msg.twist.angular.y)
    self.wz.append(msg.twist.angular.z)
    if len(self.vx) < self.window_size:
      return
    ax = lfilter(self.b, 1.0, self.vx)
    ay = lfilter(self.b, 1.0, self.vy)
    az = lfilter(self.b, 1.0, self.vz)
    alphax = lfilter(self.b, 1.0, self.wx)
    alphay = lfilter(self.b, 1.0, self.wy)
    alphaz = lfilter(self.b, 1.0, self.wz)
    last_acc = np.array([ax[-1], ay[-1], az[-1]]) * self.fs
    last_alpha = np.array([alphax[-1], alphay[-1], alphaz[-1]]) * self.fs
    acc_msg = Twist()
    acc_msg.linear = Vector3(*last_acc)
    acc_msg.angular = Vector3(*last_alpha)
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
