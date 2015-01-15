#! /usr/bin/env python
import rospy
from math import pi
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class SimpleGripper:
  joint_names = ['right_r_inner', 'right_l_inner', 'right_r_outer', 'right_l_outer', 'right_r_finger', 'right_l_finger']
  update_rate = 250 # Hz
  ratio = 1000
  R_INNER = 0; L_INNER = 1; R_OUTER = 2; L_OUTER = 3; R_FINGER = 4; L_FINGER = 5;
  def __init__(self):
    self.ns = rospy.get_namespace()
    # Setup publishers and suscribers
    self.pub_command = []
    self.commands = []
    for i, name in enumerate(self.joint_names):
      self.pub_command.append(rospy.Publisher('%s%s/command' % (self.ns, name), Float64))
      self.commands.append(0)

    rospy.Subscriber('%sright_r_inner/state' % self.ns, JointControllerState, self.cb_r_finger)
    rospy.Subscriber('%sright_l_inner/state' % self.ns, JointControllerState, self.cb_l_finger)
    rospy.Subscriber('%sright_GRIP/command' % self.ns, Float64, self.cb_command)
    
    self.gripper_command = 0
    self.r_process_value = self.l_process_value = 0
    self.r_set_point = self.l_set_point = 0
    # TODO: This should be obtained from the urdf
    self.lower_limit = -0.8
    self.upper_limit = 0.5
    rospy.loginfo('SimpleGripper ready')
        
  def execute(self):
    r = rospy.Rate(self.update_rate)
    try:
      while not rospy.is_shutdown():
        self.update()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    
  def cb_command(self, msg):
    self.gripper_command = msg.data
  
  def cb_r_finger(self, msg):
    self.r_process_value = msg.process_value
    self.r_set_point = msg.set_point
  
  def cb_l_finger(self, msg):
    self.l_process_value = msg.process_value
    self.l_set_point = msg.set_point
        
  def update(self):
    delta = self.gripper_command / self.ratio
    if self.r_set_point + delta < self.lower_limit:
      delta = 0
      self.r_set_point = self.lower_limit
    if self.r_set_point + delta > self.upper_limit:
      delta = 0
      self.r_set_point = self.upper_limit
    cmd = self.r_set_point + delta
    self.commands[self.R_INNER] = cmd
    self.commands[self.L_INNER] = -cmd
    self.commands[self.R_OUTER] = self.r_process_value
    self.commands[self.L_OUTER] = self.l_process_value
    self.commands[self.R_FINGER] = -self.r_process_value
    self.commands[self.L_FINGER] = -self.l_process_value
    for i,pub in enumerate(self.pub_command):
      pub.publish(self.commands[i])

if __name__ == '__main__':
  rospy.init_node('simple_gripper_controller')
  gripper = SimpleGripper()
  gripper.execute()
