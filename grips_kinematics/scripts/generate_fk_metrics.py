#! /usr/bin/env python
import rospy
# Messages
from grips_msgs.srv import GetStateMetrics, GetStateMetricsRequest, GetStateMetricsResponse
from grips_msgs.srv import GetJointLimits, GetJointLimitsRequest, GetJointLimitsResponse
from geometry_msgs.msg import Pose
# HDF5
import h5py, os
# Math
import numpy as np
import itertools, math

class DatabaseMetrics:
	joint_names = ['SA', 'SE', 'linkage_tr', 'WP', 'WY', 'WR']
	def __init__(self):	
		self.reachabilitystats = None
		self.reachabilitydensity3d = None
		self.reachability3d = None
		self.angledelta = 0.1
		self.metrics = None
		self.joint_states = None
		# Subscribe to metrics_service
		self.metrics_srv_name = self.read_parameter('metrics_service', '/grips/kinematic_srv/get_fk_metrics')
		self.loginfo('Waiting for %s service' % self.metrics_srv_name)
		rospy.wait_for_service(self.metrics_srv_name)
		self.metrics_srv = rospy.ServiceProxy(self.metrics_srv_name, GetStateMetrics)
		# Subscribe to limits_service
		self.limits_srv_name = self.read_parameter('limits_service', '/grips/kinematic_srv/get_joint_limits')
		self.loginfo('Waiting for %s service' % self.limits_srv_name)
		rospy.wait_for_service(self.limits_srv_name)
		self.limits_srv = rospy.ServiceProxy(self.limits_srv_name, GetJointLimits)
	
	def save_hdf5(self):
		filename = self.get_filename()
		# Try to make the dir (It should exists although)
		try:
			os.makedirs(os.path.split(filename)[0])
		except OSError, e:
			pass
		f=h5py.File(filename,'w')
		try:
			f['version'] = self.get_version()
			f['angledelta'] = self.angledelta
			f['orientations'] = self.reachabilitystats[:, :4]
			f['positions'] = self.reachabilitystats[:, 4:7]
			f['metrics'] = self.metrics
			f['joint_states'] = self.joint_states
		finally:
			if f:
				f.close()
				
	def get_version(self):
		return 5
		
	def has(self):
		return len(self.joint_states) > 0 and len(self.joint_states) > 0 and len(self.reachabilitystats) > 0
        
	def get_filename(self, database='fk_metrics'):
		database = 'fk_metrics_' + str(self.angledelta) + '_';
		folder_key = '4d0a3b5d2c41e86313f1a9bfdbc7746e'
		file_key = '27d697e7d8a999dfc3b0a3305edb1ee6'
		filename = '~/.openrave/robot.%s/%s.%s.pp' % (folder_key, database, file_key)
		return os.path.expanduser(filename)
	
	def read_parameter(self, name, default):
		if not rospy.has_param(name):
			self.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
		return rospy.get_param(name, default)
	
	def loginfo(self, msg):
		rospy.loginfo(msg)
		
	def logwarn(self, msg):
		rospy.logwarn(msg)
	
	def logerror(self, msg):
		rospy.logerr(msg)
		
	def min_max_nonzero(self, a):
		return np.min(a[np.nonzero(a)]), np.max(a[np.nonzero(a)])
		
	def array2pose(self, data):
		#~ reachabilitystats
		#~ rotation			position	solutions	
		#~ [w x y z]		[x y z]		[n]
		pose = Pose()
		pose.orientation.w = data[0]
		pose.orientation.x = data[1]
		pose.orientation.y = data[2]
		pose.orientation.z = data[3]
		pose.position.x = data[4]
		pose.position.y = data[5]
		pose.position.z = data[6]
		return pose
	
	def pose2array(self, pose):
		data = np.zeros(7)
		data[0] = pose.orientation.w
		data[1] = pose.orientation.x
		data[2] = pose.orientation.y
		data[3] = pose.orientation.z
		data[4] = pose.position.x
		data[5] = pose.position.y
		data[6] = pose.position.z
		return data
		
	
	def run(self):
		if os.path.isfile(self.get_filename()):
			answer = 'a'
			while answer not in ['y', 'Y', 'n', 'N']:
				print self.get_filename()
				answer = raw_input('The metrics file already exist, Overwrite? (y/n): ')
				if answer in ['n', 'N']:
					return
					
		self.loginfo('Generating Database fk_metrics')
		# Get the joint limits
		req = GetJointLimitsRequest()
		req.header.stamp = rospy.Time.now()
		req.header.frame_id = '/world'
		req.name = self.joint_names
		try:
			res = self.limits_srv(req)
		except rospy.ServiceException, e:
			self.logerror('Service did not process request: %s' % str(e))
		# Generate the joint states
		values = []
		self.num_poses = 1
		for i in xrange(len(res.name)):
			min_pos, max_pos = res.min_position[i], res.max_position[i]
			if np.allclose(np.zeros(2), np.array([min_pos, max_pos])):
				min_pos, max_pos = -math.pi, math.pi
			values.append(np.arange(min_pos, max_pos, self.angledelta))
			self.num_poses *= len(values[i])
		# Initialize the lists
		self.metrics = np.zeros([self.num_poses, 3])
		self.joint_states = np.zeros([self.num_poses, 6])
		self.reachabilitystats = np.zeros([self.num_poses, 7])
		self.loginfo('Processing [%d] poses. It may take a while' % self.num_poses)	
		# Process all the poses in the file using the kinematic service
		req = GetStateMetricsRequest()
		for i, row in enumerate(itertools.product(*values)):
			req.joint_states.header.stamp = rospy.Time.now()
			req.joint_states.header.frame_id = '/world'
			req.joint_states.name = self.joint_names
			req.joint_states.position = list(row)
			req.joint_states.velocity = [0] * 6
			req.joint_states.effort = [0] * 6
			try:
				res = self.metrics_srv(req)
				if res.found_group:
					self.metrics[i, 0] = res.manipulability
					self.metrics[i, 1] = res.manipulability_index
					self.metrics[i, 2] = res.max_payload
					self.joint_states[i, :] = np.array(row)
					self.reachabilitystats[i, :] = self.pose2array(res.pose)
			except rospy.ServiceException, e:
				self.logwarn('Service did not process request: %s' % str(e))
			if (i % 10000 == 0):
				self.loginfo('Evaluated: %d/%d' % (i,self.num_poses))
			# Check for shutdowns
			if rospy.is_shutdown():
				return
		self.loginfo('Done: %d metrics were calculated' % len(self.metrics))

		# Save the calculated metrics
		if self.num_poses > 0:
			self.save_hdf5()
		
if __name__ == '__main__':
	rospy.init_node('generate_fk_metrics')
	database = DatabaseMetrics()
	database.run()
