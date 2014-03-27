#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <eigen_conversions/eigen_msg.h>

// Messages
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "grips_msgs/GripsState.h"

// Flann
#include <float.h>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

// Joint limits
#include <joint_limits_interface/joint_limits_urdf.h>

class MotionControl 
{
  private:
    // Ros
    ros::NodeHandle                           nh, nh_private;
    std::vector<ros::Publisher>               control_publisher;
    ros::Publisher                            state_publisher;
    ros::Subscriber                           joint_states_sub;
    ros::Subscriber                           motion_control_sub;
    ros::Timer                                state_timer;
    // Kinematics
    robot_model_loader::RobotModelLoader      rm_loader;
    robot_model::RobotModelPtr                kinematic_model;
    robot_state::RobotStatePtr                kinematic_state;
    robot_state::JointModelGroup*             joint_model_group;
    Eigen::Affine3d                           tip_pose;
    // Joint limits
    std::map<std::string, joint_limits_interface::JointLimits> urdf_limits;
    // flann
    flann::Index<flann::L2<float> >*         index_pos;
    flann::Index<flann::L2<float> >*         index_rot;
    std::map<std::string, flann::Matrix<float> >  metrics_db;
    // Time
    ros::Time                                 last_state_print;
    ros::Time                                 last_motion_print;
    ros::Time                                 last_ik_time;
    // Misc
    std::vector<std::string>                  joint_names;
    std::string                               robot_namespace;
    std::string                               planning_group; 
    std::string                               model_frame; 
    std::string                               tip_link;
    std::string                               base_link;
    double                                   publish_frequency;
    
  public:
    MotionControl(): 
      nh_private ("~"), 
      // TODO: The robot description shouldn't be hard coded
      rm_loader("robot_description"),
      joint_model_group(0)
    { 
      // Get parameters from the server
      if (!nh_private.hasParam("planning_group")) {
        ros::param::param(std::string("~planning_group"), this->planning_group, std::string("arm"));
        ROS_WARN("Parameter [~planning_group] not found, using default: arm");
      }
      if (!nh_private.hasParam("publish_frequency")) {
        nh_private.param(std::string("publish_frequency"), this->publish_frequency, 1000.0);;
        ROS_WARN_STREAM("Parameter [~publish_frequency] not found, using default: " << this->publish_frequency << " Hz");
      }
      // get robot namespace
      this->robot_namespace = ros::this_node::getNamespace();
      if (this->robot_namespace.rfind("/") != this->robot_namespace.length()-1)
        this->robot_namespace.append("/");
      if (this->robot_namespace.length() > 1) {
        if (this->robot_namespace[0] == this->robot_namespace[1])
          this->robot_namespace.erase(0, 1);
      }
      std::string kinematic_solver;
			if (ros::param::get(this->robot_namespace + "/grips_motion_control/arm/kinematics_solver", kinematic_solver))
				ROS_INFO_STREAM("\033[94m" << "Using solver: " << kinematic_solver << "\033[0m");
			else
			{
				ROS_ERROR("Missing kinematic solver parameter");
				ros::shutdown();
				return;
			}
      // get the RobotModel loaded from urdf and srdf files
      this->kinematic_model = this->rm_loader.getModel();
      if (!this->kinematic_model) {
        ROS_ERROR("Could not load robot description");
      }
      // Get and print the name of the coordinate frame in which the transforms for this model are computed
      this->model_frame = this->kinematic_model->getModelFrame();
      // Remove the slash from model_frame
      if (this->model_frame.rfind("/") == 0)
        this->model_frame.erase(0,1);
      ROS_INFO_STREAM("Model frame: " << this->model_frame);
      // create a RobotState to keep track of the current robot pose
      this->kinematic_state.reset(new robot_state::RobotState(this->kinematic_model));
      if (!this->kinematic_state) {
        ROS_ERROR("Could not get RobotState from Model");
      }
      this->kinematic_state->setToDefaultValues();
      // Setup the joint group
      this->joint_model_group = this->kinematic_model->getJointModelGroup(this->planning_group);
      // Get the names of the joints in the arm
      const std::vector<std::string> &tmp = this->joint_model_group->getJointModelNames();
      // Get the tip and base link
      this->base_link = "base_link";
      this->tip_link = this->joint_model_group->getLinkModelNames().back();
      ROS_INFO_STREAM("Tip Link: " << this->tip_link);
      // Get the joint limits
      typedef std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it_type;
      for(it_type it = this->rm_loader.getURDF()->joints_.begin(); it != this->rm_loader.getURDF()->joints_.end(); it++)
      {
        joint_limits_interface::getJointLimits(it->second, this->urdf_limits[it->first]);
        ROS_DEBUG_STREAM("Joint " << it->first << ": " << this->urdf_limits[it->first].min_position << " " << this->urdf_limits[it->first].max_position);
        ROS_DEBUG_STREAM("Joint " << it->first << " max_velocity: " << this->urdf_limits[it->first].max_velocity);
      }
      // Setup publishers and subscribers
      std::string topic_name;
      // TODO: Here should read ONLY controllable joints (anchor is been added)
      for(std::size_t i = 1; i < tmp.size(); ++i) {
        this->joint_names.push_back(tmp[i]);
        ROS_DEBUG("Joint [%d]: %s", int(i), this->joint_names.back().c_str());
        topic_name = this->robot_namespace + this->joint_names.back() + "/command";
        this->control_publisher.push_back(nh.advertise<std_msgs::Float64>(topic_name.c_str(), 1));
      }
      topic_name = this->robot_namespace + "joint_states";
      this->joint_states_sub = nh.subscribe(topic_name.c_str(), 1, &MotionControl::cb_joint_states, this);
      
      topic_name = this->robot_namespace + "ik_motion_control";
      this->motion_control_sub = nh.subscribe(topic_name.c_str(), 1, &MotionControl::cb_ik_motion_control, this); 
      
      topic_name = this->robot_namespace + "grips_state";
      this->state_publisher = nh.advertise<grips_msgs::GripsState>(topic_name.c_str(), 1);
      
      // Setup timer for publishing state at publish_frequency
      this->state_timer = nh.createTimer(ros::Duration(1.0/this->publish_frequency), &MotionControl::publish_state, this);
      
      /* Load the previously generated metrics. Available options:
       * fk_metrics_0.15_   3600  MB
       * fk_metrics_0.2_    700.7 MB
       * fk_metrics_0.25_   190.8 MB
       * ik_metrics         32.2  MB   */
      std::string filename;
      filename = get_filename("fk_metrics_0.15_");
      ROS_INFO_STREAM("Loading [metrics database] from:\n" << filename);
      ros::Time flann_start_time = ros::Time::now();
      try {
        flann::load_from_file(this->metrics_db["positions"], filename, "positions");
        flann::load_from_file(this->metrics_db["orientations"], filename, "orientations");
        flann::load_from_file(this->metrics_db["metrics"], filename, "metrics");
        flann::load_from_file(this->metrics_db["joint_states"], filename, "joint_states");
      }
      catch (...) {
        ROS_ERROR_STREAM("Failed loading [metrics database] from:\n" << filename);
        ros::shutdown();
        return;
      }
      // Construct a randomized kd-trees. KDTreeIndexParams are the number
      // of times the tree in the index should be recursively traversed
      this->index_pos = new flann::Index<flann::L2<float> > (this->metrics_db["positions"], flann::KDTreeIndexParams(4));
      this->index_pos->buildIndex();
      double elapsed_time = (ros::Time::now() - flann_start_time).toSec();
      ROS_INFO("[metrics database] successfully loaded in %.2f seconds", elapsed_time);
      this->last_state_print = ros::Time::now();
      this->last_motion_print = ros::Time::now();
    }
    
    ~MotionControl()
    {
      // Delete flann matrices
      std::map<std::string, flann::Matrix<float> >::iterator it;
      for(it = this->metrics_db.begin(); it != this->metrics_db.end(); it++)
        delete[] this->metrics_db[it->first].ptr();
    }
    
    void publish_state(const ros::TimerEvent& _event) 
    {
      grips_msgs::GripsState state_msg;
      state_msg.header.frame_id = this->model_frame;
      tf::poseEigenToMsg(this->tip_pose, state_msg.pose);
      state_msg.header.stamp = ros::Time::now();      
      this->state_publisher.publish(state_msg);     
    }
    
    void cb_joint_states(const sensor_msgs::JointStateConstPtr& _msg) 
    {   
      std::vector<double> joint_values;
      for(std::size_t i=0; i < this->joint_names.size(); ++i) {
        for(std::size_t j=0; j < _msg->name.size(); ++j) {
          if (this->joint_names[i] == _msg->name[j])
            joint_values.push_back(_msg->position[j]);
        }
      }
      this->kinematic_state->setJointGroupPositions(this->joint_model_group, joint_values);
      // Obtain the transform of the end_effector with respect to the base_link
      Eigen::Affine3d T_model, T_end;
      T_model = this->kinematic_state->getGlobalLinkTransform(this->model_frame);
      T_end = this->kinematic_state->getGlobalLinkTransform(this->tip_link);
      this->tip_pose = T_model.inverse() * T_end;
      
      if (ros::Time::now() - this->last_state_print  >= ros::Duration(1.0))
      {
        this->last_state_print = ros::Time::now();
        // ROS_INFO_STREAM("T [with respect world]:\n" << T_end.matrix());
        ROS_DEBUG_STREAM("T [with respect base_link]:\n" << this->tip_pose.matrix());
      }
    }
    
    void cb_ik_motion_control(const geometry_msgs::PoseStampedConstPtr& _msg)
    { 
      // Validate the message frame_id
			if (_msg->header.frame_id != this->model_frame)
			{
				ROS_WARN("cb_ik_motion_control: frame_id [%s] received. Expected [%s]", _msg->header.frame_id.c_str(), this->model_frame.c_str());
				return;
			}
      // Get the latest robot state
      std::vector<double> current_joint_values;
      this->kinematic_state->copyJointGroupPositions(this->joint_model_group, current_joint_values);
      std::ostringstream current_str;     
      current_str << "current_joint_values : [";
      for(std::size_t i=0; i < this->joint_names.size(); ++i)
        current_str << current_joint_values[i] << " ";
      current_str << "]";
      // Here 1 is the number of random restart and 0.001s is the allowed time after each restart
      bool found_ik = this->kinematic_state->setFromIK(this->joint_model_group, _msg->pose, 1, 0.001);
      // Get the new joint states for the arm
      std::vector<double> new_joint_values;
      if (found_ik)
        this->kinematic_state->copyJointGroupPositions(this->joint_model_group, new_joint_values);
      else
      {
        ROS_DEBUG("Did not find IK solution");
        // Determine nn closest XYZ points in the reachability database
        int nn = 1000;
        flann::Matrix<float> query_pos(new float[3], 1, 3);
        query_pos[0][0] = _msg->pose.position.x;
        query_pos[0][1] = _msg->pose.position.y;
        query_pos[0][2] = _msg->pose.position.z;
        flann::Matrix<int> indices(new int[query_pos.rows*nn], query_pos.rows, nn);
        flann::Matrix<float> dists(new float[query_pos.rows*nn], query_pos.rows, nn);
        this->index_pos->knnSearch(query_pos, indices, dists, nn, flann::SearchParams(query_pos.cols));
        // Check that we found something
        if (indices.cols <= 0)
          return;
        std::vector<float> q_delta(nn);
        Eigen::Quaterniond q_actual(this->tip_pose.rotation());
        Eigen::Quaterniond q;
        
        for (std::size_t i=0; i < nn; i++)
        {
          // http://math.stackexchange.com/questions/90081/quaternion-distance
          q.w() = this->metrics_db["orientations"][i][0];
          q.x() = this->metrics_db["orientations"][i][1];
          q.y() = this->metrics_db["orientations"][i][2];
          q.z() = this->metrics_db["orientations"][i][3];
          q_delta[i] = 1 - pow(q.dot(q_actual), 2.0);         
          ROS_DEBUG_STREAM("Quaternion delta: " << q_delta[i]);
        }
        std::size_t min_idx = std::min_element(q_delta.begin(), q_delta.end()) - q_delta.begin();
        float min_dist = q_delta[min_idx];
        if (min_dist > 0.2)
          return;
        // Populate the new joint_values
        std::ostringstream new_str;
        new_str << "new_joint_values : [";
        for(std::size_t i=0; i < this->joint_names.size(); i++)
        {
          new_joint_values.push_back(this->metrics_db["joint_states"][indices[0][min_idx]][i]);
          new_str << new_joint_values[i] << " ";            
        }
        new_str << "]";
        this->kinematic_state->setJointGroupPositions(this->joint_model_group, new_joint_values);
        // Debugging
        if (ros::Time::now() - this->last_motion_print  >= ros::Duration(1.0))
        {
          this->last_motion_print = ros::Time::now();
          ROS_DEBUG_STREAM(current_str.str());
          ROS_DEBUG_STREAM("Index [" << min_idx << "] Distance [" << min_dist << "]");
          ROS_DEBUG_STREAM(new_str.str());
        }
      }
      // Command the robot to the new joint_values
      ROS_DEBUG("Joint names: %d, Joint values: %d", int(this->joint_names.size()), int(new_joint_values.size()));
      double elapsed_time = (ros::Time::now() - last_ik_time).toSec();
      this->last_ik_time = ros::Time::now();
      double velocity;
      std::string joint;
      for(std::size_t i=0; i < this->joint_names.size(); ++i)
      {
        // Check the max velocity
        joint = this->joint_names[i];
        if ( this->urdf_limits.find(joint) == this->urdf_limits.end() )
          continue;
        velocity = (new_joint_values[i] - current_joint_values[i])/elapsed_time;
        if (velocity > this->urdf_limits[joint].max_velocity)
          new_joint_values[i] = this->urdf_limits[joint].max_velocity*elapsed_time + current_joint_values[i];
        if (velocity < -this->urdf_limits[joint].max_velocity)
          new_joint_values[i] = -this->urdf_limits[joint].max_velocity*elapsed_time + current_joint_values[i];
        // Send the command to each joint
        std_msgs::Float64 cmd_msg;
        cmd_msg.data = new_joint_values[i];
        this->control_publisher[i].publish(cmd_msg);
        ROS_DEBUG("Joint %s: %f", this->joint_names[i].c_str(), new_joint_values[i]);
      }
      
    }
    
    std::string get_filename(const std::string& database)
    {
      std::string folder_key, file_key;
      folder_key = "4d0a3b5d2c41e86313f1a9bfdbc7746e/";
      file_key = ".27d697e7d8a999dfc3b0a3305edb1ee6.pp";
      std::ostringstream filename;
      filename << getenv("HOME") << "/.openrave/robot." << folder_key 
                << database << file_key;
      return filename.str();
    }
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "grips_motion_control");
  MotionControl mc;
  ros::spin();
  ros::shutdown();
  return 0;
}
