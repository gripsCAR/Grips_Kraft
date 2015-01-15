#!/usr/bin/env python

import rospy, os
import numpy as np
# PyKDL
import PyKDL
from tf_conversions import posemath
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_list_to_kdl
# URDF
from urdf_parser_py.urdf import URDF
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def TwistMsgToKDL(msg_twist):
  vel = PyKDL.Vector(msg_twist.linear.x, msg_twist.linear.y, msg_twist.linear.z)
  rot = PyKDL.Vector(msg_twist.angular.x, msg_twist.angular.y, msg_twist.angular.z)
  return PyKDL.Twist(vel, rot)

class JTCartesianController(object):
  def __init__(self):
    # Read the controllers parameters
    gains = read_parameter('~gains', dict())
    self.kp = joint_list_to_kdl(gains['Kp'])
    self.kd = joint_list_to_kdl(gains['Kd'])
    self.publish_rate = read_parameter('~publish_rate', 500)
    self.frame_id = read_parameter('~frame_id', 'base_link')
    self.tip_link = read_parameter('~tip_link', 'end_effector')
    # Kinematics
    self.urdf = URDF.from_parameter_server(key='robot_description')
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    # Get the joint names and limits
    self.joint_names = self.kinematics.get_joint_names()
    self.num_joints = len(self.joint_names)
    # Set-up publishers/subscribers
    self.torque_pub = dict()
    for i, name in enumerate(self.joint_names):
      self.torque_pub[name] = rospy.Publisher('/%s/command' % (name), Float64)
    rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
    rospy.Subscriber('/grips/endpoint_state', EndpointState, self.endpoint_state_cb)
    rospy.Subscriber('/grips/ik_command', PoseStamped, self.ik_command_cb)
    
    rospy.loginfo('Running Cartesian controller for Grips')
    # Start torque controller timer
    self.cart_command = None
    self.endpoint_state = None
    self.joint_states = None
    self.torque_controller_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.torque_controller_cb)
    # Shutdown hookup to clean-up before killing the script
    rospy.on_shutdown(self.shutdown)

  def endpoint_state_cb(self, msg):
    self.endpoint_state = msg

  def ik_command_cb(self, msg):
    self.cart_command = msg
    # Add stamp manually. This is a hack for console commands (rostopic pub)
    self.cart_command.header.stamp = rospy.Time.now()

  def joint_states_cb(self, msg):
    self.joint_states = msg

  def shutdown(self):
    self.torque_controller_timer.shutdown()

  def torque_controller_cb(self, event):
    if rospy.is_shutdown() or None in [self.cart_command, self.endpoint_state, self.joint_states]:
      return
    # TODO: Validate msg.header.frame_id
    
    ## Cartesian error to zero using a Jacobian transpose controller
    x_target = posemath.fromMsg(self.cart_command.pose)
    q, qd, eff = self.kinematics.extract_joint_state(self.joint_states)
    x = posemath.fromMsg(self.endpoint_state.pose)
    xdot = TwistMsgToKDL(self.endpoint_state.twist)
    # Calculate a Cartesian restoring wrench
    x_error = PyKDL.diff(x_target, x)
    wrench = np.matrix(np.zeros(6)).T
    for i in range(len(wrench)):
      wrench[i] = -(self.kp[i] * x_error[i] + self.kd[i] * xdot[i])
    # Calculate the jacobian
    J = self.kinematics.jacobian(q)
    # Convert the force into a set of joint torques. tau = J^T * wrench
    tau = J.T * wrench
    # Publish the joint_torques
    for i, name in enumerate(self.joint_names):
      self.torque_pub[name].publish(tau[i])


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  jtc = JTCartesianController()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
