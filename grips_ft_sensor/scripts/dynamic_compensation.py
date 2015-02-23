#!/usr/bin/env python
import rospy, os, collections
import numpy as np
# Filters
from scipy.signal import lfilter
# Utils
from baxter_teleop.utils import read_parameter
from grips_teleop.filters import smooth_diff
import tf.transformations as tr                 # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
# Messages
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import Twist, Vector3, WrenchStamped

def vector3_to_numpy(msg):
  return np.array([msg.x, msg.y, msg.z])
  
def quaternion_to_numpy(msg):
  return np.array([msg.x, msg.y, msg.z, msg.w])


class DynamicCompensation(object):
  def __init__(self):
    self.fs = read_parameter('/joint_state_controller/publish_rate', 1000.0)
    self.mass = read_parameter('~gripper_mass', 2.52)
    gripper_com_pose = read_parameter('~gripper_com_pose', [-0.008, 0.075, 0.0, 0.0, 0.0, 0.0])
    self.com = np.array(gripper_com_pose[:3])   # The frames should be parallel therefore there is no rotation
    self.gravity = np.array([0, 0, -9.80665])
    self.ft_frame_id = None
    
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
    self.wrench_pub = rospy.Publisher('/netft/compensated', WrenchStamped)
    rospy.Subscriber('/netft/filtered', WrenchStamped, self.netft_cb)
    rospy.Subscriber('/grips/endpoint_state', EndpointState, self.endpoint_cb)
    rospy.spin()
  
  def endpoint_cb(self, msg):
    self.vx.append(msg.twist.linear.x)
    self.vy.append(msg.twist.linear.y)
    self.vz.append(msg.twist.linear.z)
    self.wx.append(msg.twist.angular.x)
    self.wy.append(msg.twist.angular.y)
    self.wz.append(msg.twist.angular.z)
    if len(self.vx) < self.window_size or self.ft_frame_id == None:
      return
    ax = lfilter(self.b, 1.0, self.vx)
    ay = lfilter(self.b, 1.0, self.vy)
    az = lfilter(self.b, 1.0, self.vz)
    alphax = lfilter(self.b, 1.0, self.wx)
    alphay = lfilter(self.b, 1.0, self.wy)
    alphaz = lfilter(self.b, 1.0, self.wz)
    a = np.array([ax[-1], ay[-1], az[-1]]) * self.fs
    alpha = np.array([alphax[-1], alphay[-1], alphaz[-1]]) * self.fs
    # Calculate the gravity vector in the reference frame of the F/T sensor
    q = quaternion_to_numpy(msg.pose.orientation)
    g = np.squeeze(np.asarray( tr.quaternion_matrix(q)[:3,:3] * np.matrix(self.gravity).T ))
    # Estimate the force/torque to be removed from the sensor readings
    m = self.mass
    c = self.com
    w = np.array([self.wx[-1], self.wy[-1], self.wz[-1]])
    fe = m * (a - g) + np.cross(alpha, m * c) + np.cross(w, np.cross(w, m * c))
    te = np.zeros(3)
    fc = self.force - fe
    tc = self.torque - te
    # Publish results
    acc_msg = Twist()
    acc_msg.linear = Vector3(*a)
    acc_msg.angular = Vector3(*alpha)
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()
    # TODO: This should be read from the netft_cb callback
    wrench_msg.header.frame_id = self.ft_frame_id
    wrench_msg.wrench.force = Vector3(*fc)
    wrench_msg.wrench.torque = Vector3(*tc)
    try:
      self.wrench_pub.publish(wrench_msg)
      self.acc_pub.publish(acc_msg)
    except:
      pass
  
  def netft_cb(self, msg):
    self.ft_frame_id = msg.header.frame_id
    self.force = vector3_to_numpy(msg.wrench.force)
    self.torque = vector3_to_numpy(msg.wrench.torque)

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  dc = DynamicCompensation()
  rospy.loginfo('Shuting down [%s] node' % node_name)
