#!/usr/bin/env python
import rospy, os, collections
import math
import numpy as np
# Messages
from geometry_msgs.msg import Wrench, WrenchStamped
# Filter
from scipy.signal import butter, lfilter, lfilter_zi
# Utils
from baxter_teleop.utils import read_parameter


class FTSensorFilter(object):
  def __init__(self):
    # Read parameters
    fs = read_parameter('~fs', 1000)
    lowcut = read_parameter('~lowcut', 30)
    highcut = read_parameter('~highcut', 70)
    self.order = read_parameter('~order', 6)
    # Create butterworth stopband filter
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    self.b, self.a = butter(self.order, [low, high], btype='stop')
    self.window_size = 2**(math.ceil(math.log(fs/10, 2)))
    self.fx = collections.deque(maxlen=self.window_size)
    self.fy = collections.deque(maxlen=self.window_size)
    self.fz = collections.deque(maxlen=self.window_size)
    self.tx = collections.deque(maxlen=self.window_size)
    self.ty = collections.deque(maxlen=self.window_size)
    self.tz = collections.deque(maxlen=self.window_size)
    # Set-up publishers/subscribers
    self.filtered_pub = rospy.Publisher('/netft/filtered', WrenchStamped)
    rospy.Subscriber('/netft/data', WrenchStamped, self.sensor_cb)
    rospy.spin()
  
  def sensor_cb(self, msg):
    filtered_msg = WrenchStamped()
    filtered_msg.header = msg.header
    self.fx.append(msg.wrench.force.x)
    self.fy.append(msg.wrench.force.y)
    self.fz.append(msg.wrench.force.z)
    self.tx.append(msg.wrench.torque.x)
    self.ty.append(msg.wrench.torque.y)
    self.tz.append(msg.wrench.torque.z)
    if len(self.fx) < self.window_size:
      return
    fx, self.z_fx = lfilter(self.b, self.a, self.fx)
    fy, self.z_fy = lfilter(self.b, self.a, self.fy)
    fz, self.z_fz = lfilter(self.b, self.a, self.fz)
    tx, self.z_tx = lfilter(self.b, self.a, self.tx)
    ty, self.z_ty = lfilter(self.b, self.a, self.ty)
    tz, self.z_tz = lfilter(self.b, self.a, self.tz)
    f_filtered = np.array([fx[-1], fy[-1], fz[-1]])
    t_filtered = np.array([tx[-1], ty[-1], tz[-1]])
    filtered_msg.wrench.force = Vector3(*f_filtered)
    filtered_msg.wrench.torque = Vector3(*t_filtered)
    try:
      self.filtered_pub.publish(filtered_msg)
    except:
      pass

# Main
if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  ft_filter = FTSensorFilter()
  
