#! /usr/bin/env python
import rospy, time
from math import pi
from cStringIO import StringIO
# Sockets
import socket, struct
# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState

class LabviewMaster:
  SA = 0; SE = 1; EL = 2; WP = 3; WY = 4; WR = 5; GRIP = 6
  joint_names = ['left_SA', 'left_SE', 'left_linkage_tr', 'left_WP', 'left_WY', 'left_WR', 'left_GRIP',
                 'right_SA', 'right_SE', 'right_linkage_tr', 'right_WP', 'right_WY', 'right_WR', 'right_GRIP']
  def __init__(self): 
    self.ns = rospy.get_namespace()
    # Set up receiver socket
    self.read_port = 5555
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket receiving on port [%d]' % (self.read_port))
    # Set up sender socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.labview_ip = '138.100.76.191'
    self.write_port = 5050    
    rospy.loginfo('UDP Socket sending to [udp://%s:%d]' % (self.labview_ip, self.write_port))
    #Set-up publishers/subscribers
    self.command_pub = dict()
    for joint in self.joint_names:
      self.command_pub[joint] = rospy.Publisher('%s/command' % joint, Float64)
    
    while not rospy.is_shutdown():
      msg = JointState()
      data = self.recv_timeout()
      if data:
        msg.deserialize(data)
        for i, joint in enumerate(msg.name):
          self.command_pub[joint].publish(msg.position[i])
  
  def cb_joint_states(self, msg):
    file_str = StringIO()
    state.serialize(msg)
    self.write_socket.sendto(file_str.getvalue(), (self.labview_ip, self.write_port))
    
  def recv_timeout(self, timeout=0.001):
    self.read_socket.setblocking(0)
    total_data=[]
    data=''
    begin=time.time()
    while 1:
      #if you got some data, then timeout break 
      if total_data and time.time()-begin>timeout:
        break
      #if you got no data at all, wait a little longer
      elif time.time()-begin>timeout*2:
        break
      try:
        data=self.read_socket.recv(8192)
        if data:
          total_data.append(data)
          begin=time.time()
      except:
        pass
    return ''.join(total_data)
    
if __name__ == '__main__':
  rospy.init_node('labview_master')
  master = LabviewMaster()
