#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from labview_bridge import LabviewServer

class HardwareInterface(LabviewServer):
  def __init__(self): 
    LabviewServer.__init__(self)
    self.gripper_cmd = -10
    # Set-up publishers/subscribers
    rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
    rospy.Subscriber('/grips/gripper/command', Float64, self.gripper_cmd_cb)
    self.el_offset = math.atan2(29.845, 123.4444)
    rospy.spin()

  def gripper_cmd_cb(self, msg):
    self.gripper_cmd = msg.data

  def joint_states_cb(self, msg):
    cmd_msg = JointState()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.header.frame_id = 'base_link'
    cmd_msg.name = list(msg.name)
    cmd_msg.position = list(msg.position)
    # Convert the value from linkage_tr to EL
    SE = msg.position[msg.name.index('SE')]
    TR = msg.position[msg.name.index('linkage_tr')]
    EL  = TR + SE + math.pi/2.0 - self.el_offset
    cmd_msg.position[msg.name.index('linkage_tr')] = EL
    cmd_msg.name[msg.name.index('linkage_tr')] = 'EL'
    # Add gripper command
    cmd_msg.name.append('gripper')
    cmd_msg.position.append(self.gripper_cmd)
    # Serialize cmd_msg
    file_str = StringIO()
    cmd_msg.serialize(file_str)
    # Send over udp the cmd_msg sensor_msgs/JointState
    self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = HardwareInterface()
