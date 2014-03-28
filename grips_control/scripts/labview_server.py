#! /usr/bin/env python
import rospy, time, math
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''

class LabviewServer:
  joint_names = ['SA', 'SE', 'WP', 'WR', 'WY', 'l_finger', 'l_inner', 'l_outer', 'linkage_bl', 'linkage_tl', 'linkage_tr', 'r_finger', 'r_inner', 'r_outer']
  cmd_names = ['SA','SE','EL','WP','WY','WR','GRIP']
  def __init__(self): 
    # Read all the parameters from the parameter server
    self.publish_frequency = self.read_parameter('~publish_frequency', 1000.0)
    # UDP
    self.read_port = int(self.read_parameter('~read_port', 5555))
    self.write_ip = self.read_parameter('~write_ip', '192.168.0.4')
    self.write_port = int(self.read_parameter('~write_port', 5050))
    # Topics
    self.js_topic = self.read_parameter('~joint_states_topic', '/joint_states')
    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))
    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.loginfo('UDP Socket sending to [udp://%s:%d]' % (self.write_ip, self.write_port))
    # Set-up publishers/subscribers
    self.joint_pub = rospy.Publisher(self.js_topic, JointState)
    tmp = list(self.cmd_names)
    rospy.Subscriber('/SA/command', Float64, self.cb_SA)
    rospy.Subscriber('/SE/command', Float64, self.cb_SE)
    rospy.Subscriber('/linkage_tr/command', Float64, self.cb_EL)
    rospy.Subscriber('/WP/command', Float64, self.cb_WP)
    rospy.Subscriber('/WY/command', Float64, self.cb_WY)
    rospy.Subscriber('/WR/command', Float64, self.cb_WR)
    rospy.Subscriber('/GRIP/command', Float64, self.cb_GRIP)
    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
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
  
  def execute(self):
    while not rospy.is_shutdown():
      recv_msg = JointState()
      data = server.recv_timeout()
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

  def shutdown_hook(self):
    # Stop the publisher timer
    self.cmd_timer.shutdown()
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    
  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
    
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
  rospy.init_node('labview_server')
  server = LabviewServer()
  server.execute()
