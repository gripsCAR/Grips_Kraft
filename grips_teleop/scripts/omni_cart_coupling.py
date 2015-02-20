#!/usr/bin/env python
import rospy, os, math
import numpy as np
# Baxter Interface for the grippers
import baxter_interface
from baxter_interface import CHECK_VERSION
# Messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Wrench
from omni_msgs.msg import OmniButtonEvent
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
# Services
from std_srvs.srv import Empty
# Quaternions tools
import PyKDL
# Utils
from baxter_teleop.utils import read_parameter
from math import sqrt, pi


GREY_BUTTON = 0
WHITE_BUTTON = 1
GRIPPER_RATIO = 10


class CartCoupling(object):
  def __init__(self):
    self.roll_angle = 0
    self.gripper_cmd = -GRIPPER_RATIO
    self.q0 = PyKDL.Rotation.Quaternion(-math.sqrt(2)/2.0, 0.0, 0.0, math.sqrt(2)/2.0)
    # Set-up publishers/subscribers
    self.locked_pub = rospy.Publisher('/omni/locked', Bool)
    self.gripper_pub = rospy.Publisher('/grips/gripper/command', Float64)
    self.ik_cmd_pub = rospy.Publisher('/grips/ik_command', PoseStamped)
    rospy.Subscriber('/grips/raw_ik_command', PoseStamped, self.pose_cb)
    self.prev_buttons = [0] * 2
    self.buttons = [False] * 2
    self.buttons[WHITE_BUTTON] = True
    rospy.Subscriber('/omni/button', OmniButtonEvent, self.buttons_cb)
    rospy.Subscriber('/omni/joint_states', JointState, self.joint_states_cb)
    rospy.spin()
  
  def buttons_cb(self, msg):
    button_states = [msg.grey_button, msg.white_button]
    # Check that any button was pressed / released
    for i, previous in enumerate(self.prev_buttons):
      if (previous != button_states[i]) and button_states[i] == 1:
        self.buttons[i] = not self.buttons[i]
    # Open or close the gripper
    if self.buttons[GREY_BUTTON]:   # Close
      self.gripper_cmd = GRIPPER_RATIO
    else:                           # Open
      self.gripper_cmd = -GRIPPER_RATIO
    self.gripper_pub.publish(Float64(self.gripper_cmd))
  
  def joint_states_cb(self, msg):
    idx = msg.name.index('roll')
    self.roll_angle = -msg.position[idx]

  def pose_cb(self, msg):
    self.locked_pub.publish(Bool(self.buttons[WHITE_BUTTON]))
    cmd_msg = msg
    q_roll = PyKDL.Rotation.RotY(self.roll_angle)
    q = (self.q0 * q_roll).GetQuaternion()
    cmd_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    if not self.buttons[WHITE_BUTTON]:
      try:
        self.ik_cmd_pub.publish(cmd_msg)
      except:
        pass


# Main
if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  cc = CartCoupling()
  
