#!/usr/bin/env python
import rospy, os
import math
import numpy as np
# Messages
from geometry_msgs.msg import Wrench, WrenchStamped
# Filter
from scipy.signal import butter, lfilter, lfilter_zi
# Utils
from baxter_teleop.utils import read_parameter

def fromWrench(wrench):
  return [wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z]

def toWrench(vector):
  wrench = Wrench()
  wrench.force.x = vector[0]
  wrench.force.y = vector[1]
  wrench.force.z = vector[2]
  wrench.torque.x = vector[3]
  wrench.torque.y = vector[4]
  wrench.torque.z = vector[5] 
  return wrench


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
    self.fx = []
    self.fy = []
    self.fz = []
    self.tx = []
    self.ty = []
    self.tz = []
    self.z_fx = lfilter_zi(self.b, self.a)
    self.z_fy = lfilter_zi(self.b, self.a)
    self.z_fz = lfilter_zi(self.b, self.a)
    self.z_tx = lfilter_zi(self.b, self.a)
    self.z_ty = lfilter_zi(self.b, self.a)
    self.z_tz = lfilter_zi(self.b, self.a)
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
    fx, self.z_fx = lfilter(self.b, self.a, self.fx, zi=np.array(self.z_fx))
    fy, self.z_fy = lfilter(self.b, self.a, self.fy, zi=np.array(self.z_fy))
    fz, self.z_fz = lfilter(self.b, self.a, self.fz, zi=np.array(self.z_fz))
    tx, self.z_tx = lfilter(self.b, self.a, self.tx, zi=np.array(self.z_tx))
    ty, self.z_ty = lfilter(self.b, self.a, self.ty, zi=np.array(self.z_ty))
    tz, self.z_tz = lfilter(self.b, self.a, self.tz, zi=np.array(self.z_tz))
    self.fx.pop(0)
    self.fy.pop(0)
    self.fz.pop(0)
    self.tx.pop(0)
    self.ty.pop(0)
    self.tz.pop(0)
    filtered_msg.wrench.force.x = fx[-1]
    filtered_msg.wrench.force.y = fy[-1]
    filtered_msg.wrench.force.z = fz[-1]
    filtered_msg.wrench.torque.x = tx[-1]
    filtered_msg.wrench.torque.y = ty[-1]
    filtered_msg.wrench.torque.z = tz[-1]
    try:
      self.filtered_pub.publish(filtered_msg)
    except:
      pass

# Main
if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  ft_filter = FTSensorFilter()
  
