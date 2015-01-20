#!/usr/bin/env python

import rospy, os
# PyKDL
import PyKDL
from hrl_geom.pose_converter import PoseConv
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_list_to_kdl
# URDF
from urdf_parser_py.urdf import URDF
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3

def TwistKDLToMsg(kdl_twist):
  msg_twist = Twist()
  msg_twist.linear = Vector3(*kdl_twist.vel)
  msg_twist.angular = Vector3(*kdl_twist.rot)
  return msg_twist

class StateNode(object):
  def __init__(self):
    # Read parameters
    self.frame_id = read_parameter('~frame_id', 'base_link')
    self.tip_link = read_parameter('~tip_link', 'end_effector')
    # Kinematics
    self.urdf = URDF.from_parameter_server(key='robot_description')
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    self.fk_vel_solver = PyKDL.ChainFkSolverVel_recursive(self.kinematics.chain)
    # Set-up publishers/subscribers
    self.state_pub = rospy.Publisher('/grips/endpoint_state', EndpointState)
    rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
  
  def joint_states_cb(self, msg):
    state_msg = EndpointState()
    # Forward Position Kinematics
    q, qd, eff = self.kinematics.extract_joint_state(msg)
    T06 = self.kinematics.forward(q)
    state_msg.pose = PoseConv.to_pose_msg(T06)
    # Forward Velocity Kinematics
    end_frame = PyKDL.FrameVel()
    q_vel = PyKDL.JntArrayVel(joint_list_to_kdl(q), joint_list_to_kdl(qd))
    self.fk_vel_solver.JntToCart(q_vel, end_frame)
    state_msg.twist = TwistKDLToMsg(end_frame.GetTwist())
    state_msg.header.frame_id = self.frame_id
    state_msg.header.stamp = rospy.Time.now()
    try:
      self.state_pub.publish(state_msg)
    except:
      pass
    

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  sn = StateNode()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
