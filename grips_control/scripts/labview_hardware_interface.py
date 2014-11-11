#! /usr/bin/env python
import rospy, time, math
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from labview_bridge import LabviewServer

class HardwareInterface(LabviewServer):
  joint_names = ['SA', 'SE', 'WP', 'WR', 'WY', 'l_finger', 'l_inner', 'l_outer', 'linkage_bl', 'linkage_tl', 'linkage_tr', 'r_finger', 'r_inner', 'r_outer']
  cmd_names = ['SA','SE','EL','WP','WY','WR','GRIP']
  def __init__(self): 
    LabviewServer.__init__(self)
    # Topics
    self.js_topic = self.read_parameter('~joint_states_topic', '%sjoint_states' % self.ns)
    # Set-up publishers/subscribers
    self.joint_pub = rospy.Publisher(self.js_topic, JointState)
    tmp = list(self.cmd_names)
    rospy.Subscriber('%sSA/command' % self.ns, Float64, self.cb_SA)
    rospy.Subscriber('%sSE/command' % self.ns, Float64, self.cb_SE)
    rospy.Subscriber('%slinkage_tr/command' % self.ns, Float64, self.cb_EL)
    rospy.Subscriber('%sWP/command' % self.ns, Float64, self.cb_WP)
    rospy.Subscriber('%sWY/command' % self.ns, Float64, self.cb_WY)
    rospy.Subscriber('%sWR/command' % self.ns, Float64, self.cb_WR)
    rospy.Subscriber('%sGRIP/command' % self.ns, Float64, self.cb_GRIP)
    # Initial values
    self.cmd_msg = JointState()
    self.cmd_msg.name = self.cmd_names
    self.cmd_msg.position = [None] * len(self.cmd_names)
    self.joint_msg = JointState()
    self.joint_msg.name = self.joint_names
    self.joint_msg.position = [0] * len(self.joint_names)
    self.joint_msg.velocity = [0] * len(self.joint_names)
    self.joint_msg.effort = [0] * len(self.joint_names)
    # Start the timer that will send the commands to labview 
    self.cmd_timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.cb_write_commands)
    
  def shutdown_hook(self):
    # Parent class clean-up
    LabviewServer.shutdown_hook(self)
    # Stop the publisher timer
    self.cmd_timer.shutdown()
  
  def execute(self):
    while not rospy.is_shutdown():
      recv_msg = JointState()
      data = self.recv_timeout()
      if data:
        # Serialize received UDP message
        recv_msg.deserialize(data)
        # Add header for robot_state_publisher
        self.joint_msg.header.seq = recv_msg.header.seq
        self.joint_msg.header.stamp = rospy.Time.now()
        # Populate the joint_msg with the receiver positions
        for j, name in enumerate(self.joint_msg.name):
          if name in recv_msg.name:
            self.joint_msg.position[j] = recv_msg.position[recv_msg.name.index(name)]
        if 'EL' in recv_msg.name:
          EL = recv_msg.position[recv_msg.name.index('EL')]
          SE = recv_msg.position[recv_msg.name.index('SE')]
          TR = (EL - SE - math.pi)
          self.joint_msg.position[self.joint_names.index('linkage_tr')] = TR
          self.joint_msg.position[self.joint_names.index('linkage_tl')] = -TR
          self.joint_msg.position[self.joint_names.index('linkage_bl')] = TR
        self.joint_pub.publish(self.joint_msg)
        
  def cb_write_commands(self, event):
    if None in self.cmd_msg.position:
      rospy.logdebug('[cmd_msg] has non populate positions')
      return
    # Serialize cmd_msg
    file_str = StringIO()
    self.cmd_msg.serialize(file_str)
    # Send over udp the cmd_msg sensor_msgs/JointState
    self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))
    
  def cb_SA(self, msg):      
    self.cmd_msg.position[0] = msg.data
  
  def cb_SE(self, msg):
    self.cmd_msg.position[1] = msg.data
  
  def cb_EL(self, msg):
    # Convert the value from linkage_tr to EL
    if self.cmd_msg.position[1] == None:
      return
    TR = msg.data;
    SE = self.cmd_msg.position[1];
    self.cmd_msg.position[2] = TR + SE + math.pi
  
  def cb_WP(self, msg):
    self.cmd_msg.position[3] = msg.data
    
  def cb_WY(self, msg):
    self.cmd_msg.position[4] = msg.data
  
  def cb_WR(self, msg):
    self.cmd_msg.position[5] = msg.data
  
  def cb_GRIP(self, msg):
    self.cmd_msg.position[6] = msg.data


if __name__ == '__main__':
  rospy.init_node('labview_hardware_interface')
  interface = HardwareInterface()
  interface.execute()
