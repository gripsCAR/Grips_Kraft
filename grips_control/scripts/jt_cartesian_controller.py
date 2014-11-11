#!/usr/bin/env python

import rospy, os
import numpy as np
# PyKDL
import PyKDL
from tf_conversions import posemath
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_kdl_to_list, joint_list_to_kdl, kdl_to_mat
# URDF
from urdf_parser_py.urdf import URDF
# Baxter Python Interface
import baxter_interface
from baxter_faces import FaceImage
# Utils
from baxter_teleop.utils import read_parameter, baxter_to_kdl_frame, baxter_to_kdl_twist
from math import pi
# Messages
from baxter_core_msgs.msg import EndEffectorState
from geometry_msgs.msg import PoseStamped, Vector3, Wrench
from std_msgs.msg import Empty, UInt16
from takktile_ros.msg import Touch


class JTCartesianController(object):
  def __init__(self):
    limb = read_parameter('~limb', 'right')
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    # Read the controllers parameters
    gains = read_parameter('~gains', dict())
    self.kp = joint_list_to_kdl(gains['Kp'])
    self.kd = joint_list_to_kdl(gains['Kd'])
    self.publish_rate = read_parameter('~publish_rate', 500)
    self.frame_id = read_parameter('~frame_id', 'base')
    self.timeout = read_parameter('~timeout', 0.02)       # seconds
    # Set-up baxter interface
    self.arm_interface = baxter_interface.Limb(limb)
    # set_joint_torques must be commanded at a rate great than the timeout specified by set_command_timeout.
    # If the timeout is exceeded before a new set_joint_velocities command is received, the controller will switch
    # modes back to position mode for safety purposes.
    self.arm_interface.set_command_timeout(self.timeout)
    # Baxter kinematics
    self.urdf = URDF.from_parameter_server(key='robot_description')
    self.tip_link = '%s_gripper' % limb
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    self.fk_vel_solver = PyKDL.ChainFkSolverVel_recursive(self.kinematics.chain)
    # Adjust the publish rate of baxter's state
    joint_state_pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
    # Get the joint names and limits
    self.joint_names = self.arm_interface.joint_names()
    self.num_joints = len(self.joint_names)
    self.lower_limits = np.zeros(self.num_joints)
    self.upper_limits = np.zeros(self.num_joints)
    # KDL joint may be in a different order than expected
    kdl_lower_limits, kdl_upper_limits = self.kinematics.get_joint_limits()
    for i, name in enumerate(self.kinematics.get_joint_names()):
      if name in self.joint_names:
        idx = self.joint_names.index(name)
        self.lower_limits[idx] = kdl_lower_limits[i]
        self.upper_limits[idx] = kdl_upper_limits[i]
    self.middle_values = (self.upper_limits + self.lower_limits) / 2.0
    # Move the arm to a neutral position
    #~ self.arm_interface.move_to_neutral(timeout=10.0)
    #~ neutral_pos = [-math.pi/4, 0.0, 0.0, 0.0, 0.0, math.pi/2, -math.pi/2]
    neutral_pos = [1.0, -0.57, -1.2, 1.3, 0.86, 1.4, -0.776]
    #~ neutral_pos = [pi/4, -pi/3, 0.0, pi/2, 0.0, pi/3, 0.0]
    neutral_cmd = dict()
    for i, name in enumerate(self.joint_names):
      if name in ['left_s0', 'left_w2']:
        neutral_cmd[name] = neutral_pos[i] * -1.0
      else:
        neutral_cmd[name] = neutral_pos[i]
    self.arm_interface.move_to_joint_positions(neutral_cmd, timeout=10.0)
    # Baxter faces
    self.limb = limb
    self.face = FaceImage()
    self.face.set_image('happy')
    # Set-up publishers/subscribers
    self.suppress_grav_comp = rospy.Publisher('/robot/limb/%s/suppress_gravity_compensation' % limb, Empty)
    joint_state_pub_rate.publish(int(self.publish_rate))
    self.feedback_pub = rospy.Publisher('/takktile/force_feedback', Wrench)
    rospy.Subscriber('/baxter/%s_ik_command' % limb, PoseStamped, self.ik_command_cb)
    self.gripper_closed = False
    rospy.Subscriber('/takktile/calibrated', Touch, self.takktile_cb)
    rospy.Subscriber('/robot/end_effector/%s_gripper/state' % limb, EndEffectorState, self.end_effector_cb)
    
    rospy.loginfo('Running Cartesian controller for the %s arm' % limb)
    # Start torque controller timer
    self.cart_command = None
    self.torque_controller_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.torque_controller_cb)
    # Shutdown hookup to clean-up before killing the script
    rospy.on_shutdown(self.shutdown)
    
  def end_effector_cb(self, msg):
    self.gripper_closed = msg.position < 80.0

  def get_joint_angles_array(self):
    out = np.zeros(self.num_joints)
    joint_angles = self.arm_interface.joint_angles()
    for i, name in enumerate(self.joint_names):
      out[i] = joint_angles[name]
    return out
  
  def ik_command_cb(self, msg):
    if self.cart_command == None:
      self.face.set_image('thinking_%s' % self.limb)
    self.cart_command = msg
    # Add stamp because the console app doesn't use it
    self.cart_command.header.stamp = rospy.Time.now()
  
  def shutdown(self):
    self.face.set_image('indifferent')
    # This order mathers!
    self.arm_interface.exit_control_mode()
    self.torque_controller_timer.shutdown()
  
  def takktile_cb(self, msg):
    if self.gripper_closed:
      y = 0
    else:
      y = 0.003 * (sum(msg.pressure[0:4]) - sum(msg.pressure[6:10]))
    z = 0.006 * (msg.pressure[4] + msg.pressure[10])
    # Changes the reference frame of the force vector
    T = baxter_to_kdl_frame(self.arm_interface.endpoint_pose())
    force = T.M * PyKDL.Vector(0, y, z)
    # Populate the wrench message
    wrench = Wrench()
    wrench.force.x = force.x()
    wrench.force.y = force.y()
    wrench.force.z = force.z()
    try:
      self.feedback_pub.publish(wrench)
    except:
      pass

  def torque_controller_cb(self, event):
    if rospy.is_shutdown() or self.cart_command == None:
      return
    elapsed_time = rospy.Time.now() - self.cart_command.header.stamp
    if elapsed_time.to_sec() > self.timeout:
      return
    # TODO: Validate msg.header.frame_id
    
    ## Cartesian error to zero using a Jacobian transpose controller
    x_target = posemath.fromMsg(self.cart_command.pose)
    q = self.get_joint_angles_array()
    x = baxter_to_kdl_frame(self.arm_interface.endpoint_pose())
    xdot = baxter_to_kdl_twist(self.arm_interface.endpoint_velocity())
    # Calculate a Cartesian restoring wrench
    x_error = PyKDL.diff(x_target, x)
    wrench = np.matrix(np.zeros(6)).T
    for i in range(len(wrench)):
      wrench[i] = -(self.kp[i] * x_error[i] + self.kd[i] * xdot[i])
    # Calculate the jacobian
    J = self.kinematics.jacobian(q)
    # Convert the force into a set of joint torques. tau = J^T * wrench
    tau = J.T * wrench

    # Populate the joint_torques in baxter API format (dictionary)
    joint_torques = dict()
    for i, name in enumerate(self.joint_names):
      joint_torques[name] = tau[i]
    self.arm_interface.set_joint_torques(joint_torques)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  jtc = JTCartesianController()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
