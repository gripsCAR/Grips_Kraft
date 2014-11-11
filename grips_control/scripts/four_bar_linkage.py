#! /usr/bin/env python
import rospy
from math import pi
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class FourBarLinkage:
  joint_names = ['linkage_tl', 'linkage_bl']
  TL = 0; BL = 1;
  def __init__(self):
    self.ns = rospy.get_namespace()
    # Setup publishers and suscribers
    self.pub_command = []
    self.commands = []
    for i, name in enumerate(self.joint_names):
      self.pub_command.append(rospy.Publisher('%s%s/command' % (self.ns, name), Float64))
      self.commands.append(0)

    rospy.Subscriber('%slinkage_tr/state' % self.ns, JointControllerState, self.cb_tr)
    
    # Wait for commands in the linkage_tr controller
    self.tr_angle = None
    try:
      while not rospy.is_shutdown():
        if self.tr_angle == None:
          rospy.sleep(0.01)
        else:
          rospy.loginfo('FourBarLinkage ready')
          break
    except rospy.exceptions.ROSInterruptException:
      pass
  
  def cb_tr(self, msg):
    self.commands[self.TL] = -msg.process_value
    self.commands[self.BL] = msg.process_value
    for i, pub in enumerate(self.pub_command):
      pub.publish(self.commands[i])
    

if __name__ == '__main__':
  rospy.init_node('four_bar_controller')
  linkage = FourBarLinkage()
  rospy.spin()
