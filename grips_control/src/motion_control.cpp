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

// Grips kinematic Interface
#include <grips_kinematics/kinematic_interface.hpp>

using namespace grips_kinematics;

class MotionControl 
{
  private:
    // Ros
    ros::NodeHandle                           nh_, nh_private_;
    std::vector<ros::Publisher>               control_publisher_;
    ros::Publisher                            state_publisher_;
    ros::Publisher                            pose_publisher_;
    ros::Subscriber                           joint_states_sub_;
    ros::Subscriber                           motion_control_sub_;
    ros::Timer                                state_timer_;
    ros::Timer                                pose_timer_;
    // Kinematics
    KinematicInterfacePtr                     fk_kinematics_;
    KinematicInterfacePtr                     ik_kinematics_;
    Eigen::Affine3d                           end_effector_pose_;
    std::map<std::string, joint_limits_interface::JointLimits> urdf_limits_;
    // flann
    flann::Index<flann::L2<float> >*          position_index_;
    //~ flann::Index<flann::L2<float> >*          index_rot_;
    std::map<std::string, flann::Matrix<float> >  metrics_db_;
    // Time
    ros::Time                                 last_state_print_;
    ros::Time                                 last_motion_print_;
    ros::Time                                 last_ik_time_;
    // Misc
    std::vector<std::string>                  joint_names_;
    std::string                               robot_namespace_;
    std::string                               model_frame_; 
    double                                    publish_frequency_;
    double                                    position_error_;
    
  public:
    MotionControl(): 
      nh_private_("~")
    { 
      // Get parameters from the server
      std::string database;
      nh_private_.param(std::string("publish_frequency"), publish_frequency_, 1000.0);
      nh_private_.param(std::string("metrics_database"), database, std::string("ik_metrics"));
      nh_private_.param(std::string("position_error"), position_error_, 0.05);
      if (!nh_private_.hasParam("publish_frequency"))
        ROS_WARN_STREAM("Parameter [~publish_frequency] not found, using default: " << publish_frequency_ << " Hz");      
      if (!nh_private_.hasParam("metrics_database"))
        ROS_WARN_STREAM("Parameter [~metrics_database] not found, using default: " << database);
      if (!nh_private_.hasParam("position_error"))
        ROS_WARN_STREAM("Parameter [~position_error] not found, using default: " << position_error_ << " m.");
      
      // Get robot namespace
      robot_namespace_ = ros::this_node::getNamespace();
      if (robot_namespace_.rfind("/") != robot_namespace_.length()-1)
        robot_namespace_.append("/");
      if (robot_namespace_.length() > 1) 
      {
        if (robot_namespace_[0] == robot_namespace_[1])
          robot_namespace_.erase(0, 1);
      }
      // Kinematic interfaces
      fk_kinematics_.reset(new KinematicInterface());
      ik_kinematics_.reset(new KinematicInterface());
      joint_names_ = fk_kinematics_->getActiveJointModelNames();
      model_frame_ = fk_kinematics_->getModelFrame();
      urdf_limits_ = fk_kinematics_->getJointLimits();
      
      // Setup publishers and subscribers
      std::string topic_name;
      for(std::size_t i = 0; i < joint_names_.size(); ++i) {
        ROS_DEBUG("Joint [%d]: %s", int(i), joint_names_[i].c_str());
        topic_name = robot_namespace_ + joint_names_[i] + "/command";
        control_publisher_.push_back(nh_.advertise<std_msgs::Float64>(topic_name.c_str(), 1));
      }
      topic_name = robot_namespace_ + "joint_states";
      joint_states_sub_ = nh_.subscribe(topic_name.c_str(), 1, &MotionControl::jointStatesCB, this);
      topic_name = robot_namespace_ + "ik_command";
      motion_control_sub_ = nh_.subscribe(topic_name.c_str(), 1, &MotionControl::ikCommandCB, this); 
      topic_name = robot_namespace_ + "state";
      state_publisher_ = nh_.advertise<grips_msgs::GripsState>(topic_name.c_str(), 1);
      topic_name = robot_namespace_ + "pose";
      pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name.c_str(), 1);
      
      // Setup timer for publishing state at publish_frequency
      state_timer_ = nh_.createTimer(ros::Duration(1.0/publish_frequency_), &MotionControl::publishState, this);
      // Setup timer for publishing pose at 60 Hz
      pose_timer_ = nh_.createTimer(ros::Duration(1.0/60), &MotionControl::publishPose, this);
      
      /* Load the previously generated metrics. Available options:
       * ik_metrics         684  MB    ~30    sec. */
      std::string filename;
      filename = getFilename(database);
      ROS_INFO_STREAM("Loading [metrics database] from:\n" << filename);
      ros::Time flann_start_time = ros::Time::now();
      try {
        flann::load_from_file(metrics_db_["positions"], filename, "positions");
        flann::load_from_file(metrics_db_["orientations"], filename, "orientations");
        flann::load_from_file(metrics_db_["metrics"], filename, "metrics");
        flann::load_from_file(metrics_db_["joint_states"], filename, "joint_states");
      }
      catch (...) {
        ROS_ERROR_STREAM("Failed loading [metrics database] from:\n" << filename);
        ros::shutdown();
        return;
      }
      // construct an randomized kd-tree index using 4 kd-trees
      std::ostringstream index_file;
      index_file << getFolderName() << "ik_metrics_index.dat";
      flann::SavedIndexParams saved_params = flann::SavedIndexParams(index_file.str());
      position_index_ = new flann::Index<flann::L2<float> > (metrics_db_["positions"], saved_params);
      
      //position_index_ = new flann::Index<flann::L2<float> > (metrics_db_["positions"], flann::KDTreeIndexParams(4));
      //position_index_->buildIndex();
      double elapsed_time = (ros::Time::now() - flann_start_time).toSec();
      ROS_INFO("[metrics database] successfully loaded in %.2f seconds", elapsed_time);
      last_state_print_ = ros::Time::now();
      last_motion_print_ = ros::Time::now();
    }
    
    ~MotionControl()
    {
      // Delete flann matrices
      std::map<std::string, flann::Matrix<float> >::iterator it;
      for(it = metrics_db_.begin(); it != metrics_db_.end(); it++)
        delete[] metrics_db_[it->first].ptr();
    }
    
    void publishPose(const ros::TimerEvent& _event) 
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = model_frame_;
      tf::poseEigenToMsg(end_effector_pose_, pose_msg.pose);
      pose_msg.header.stamp = ros::Time::now();      
      // publish pose only (for rviz visualization)
      pose_publisher_.publish(pose_msg);
    }
    
    void publishState(const ros::TimerEvent& _event) 
    {
      grips_msgs::GripsState state_msg;
      state_msg.header.frame_id = model_frame_;
      tf::poseEigenToMsg(end_effector_pose_, state_msg.pose);
      state_msg.header.stamp = ros::Time::now();
      // publish the GripsState msg
      state_publisher_.publish(state_msg);
    }
    
    void jointStatesCB(const sensor_msgs::JointStateConstPtr& _msg) 
    {   
      std::vector<double> joint_values;
      for(std::size_t i=0; i < joint_names_.size(); ++i) {
        for(std::size_t j=0; j < _msg->name.size(); ++j) {
          if (joint_names_[i] == _msg->name[j])
          {
            joint_values.push_back(_msg->position[j]);
            break;
          }
        }
      }
      // Do FK
      fk_kinematics_->setJointPositions(joint_values);
      Eigen::Affine3d T_base, T_end;
      // Obtain the transform of the end_effector with respect to the model_frame_
      //~ T_base = fk_kinematics_->getGlobalLinkTransform(model_frame_);
      //~ T_end = fk_kinematics_->getEndEffectorTransform();
      //~ end_effector_pose_ = T_base.inverse() * T_end;
      end_effector_pose_ = fk_kinematics_->getEndEffectorTransform();
      
      // Debug
      if (ros::Time::now() - last_state_print_  >= ros::Duration(1.0))
      {
        last_state_print_ = ros::Time::now();
        ROS_DEBUG_STREAM("T [with respect base_link]:\n" << end_effector_pose_.matrix());
      }
    }
    
    void ikCommandCB(const geometry_msgs::PoseStampedConstPtr& _msg)
    { 
      // Validate the message frame_id
      if (_msg->header.frame_id != model_frame_)
      {
        ROS_WARN("ikCommandCB: frame_id [%s] received. Expected [%s]", _msg->header.frame_id.c_str(), model_frame_.c_str());
        return;
      }      
      // Get the latest robot state
      std::vector<double> current_joint_values;
      ik_kinematics_->getJointPositions(current_joint_values);
      std::ostringstream current_str;     
      current_str << "current_joint_values : [";
      for(std::size_t i=0; i < joint_names_.size(); ++i)
        current_str << current_joint_values[i] << " ";
      current_str << "]";
      // Here 1 is the number of random restart and 0.001s is the allowed time after each restart
      bool found_ik = ik_kinematics_->setEndEffectorPose(_msg->pose, 1, 0.001);
      // Get the new joint states for the arm
      std::vector<double> new_joint_values;
      if (found_ik)
        ik_kinematics_->getJointPositions(new_joint_values);
      else
      {
        // Check that the IK command has changed enought
        Eigen::Affine3d goal_pose, current_pose = ik_kinematics_->getEndEffectorTransform();
        tf::poseMsgToEigen(_msg->pose, goal_pose);
        double x_dist = goal_pose.translation().x() - current_pose.translation().x();
        double y_dist = goal_pose.translation().y() - current_pose.translation().y();
        double z_dist = goal_pose.translation().z() - current_pose.translation().z();
        double xyz_dist = sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2));
        
        if (xyz_dist < position_error_)
          return;
        
        ROS_DEBUG("Did not find IK solution");
        // Determine nn closest XYZ points in the reachability database
        int max_nn = 1000, nearest_neighbors;
        flann::Matrix<float> query_pos(new float[3], 1, 3);
        query_pos[0][0] = _msg->pose.position.x;
        query_pos[0][1] = _msg->pose.position.y;
        query_pos[0][2] = _msg->pose.position.z;
        flann::Matrix<int> indices(new int[query_pos.rows*max_nn], query_pos.rows, max_nn);
        flann::Matrix<float> dists(new float[query_pos.rows*max_nn], query_pos.rows, max_nn);
        // do a knn search, using 128 checks
        // this->index_pos->knnSearch(query_pos, indices, dists, nn, flann::SearchParams(128));
        nearest_neighbors = indices.cols;
        float radius = pow(position_error_, 2);
        nearest_neighbors = position_index_->radiusSearch(query_pos, indices, dists, radius, flann::SearchParams(128));
        // Check that we found something
        if (nearest_neighbors <= 0) {
          ROS_INFO_THROTTLE(60, "Didn't find any shit. Query: [%.3f, %.3f, %.3f]", _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
          return; }
        nearest_neighbors = fmin(nearest_neighbors, max_nn);
        std::vector<float> score(nearest_neighbors), d_q(nearest_neighbors), 
                           d_j(nearest_neighbors), d_xyz(nearest_neighbors);
        Eigen::Quaterniond q_actual(end_effector_pose_.rotation());
        Eigen::Quaterniond q;        
        for (std::size_t i=0; i < nearest_neighbors; ++i)
        {
          // http://math.stackexchange.com/questions/90081/quaternion-distance
          q.w() = metrics_db_["orientations"][i][0];
          q.x() = metrics_db_["orientations"][i][1];
          q.y() = metrics_db_["orientations"][i][2];
          q.z() = metrics_db_["orientations"][i][3];
          d_q[i] = 1 - pow(q.dot(q_actual), 2.0);
          d_xyz[i] = sqrtf(dists[0][i]);
          d_j[i] = 0;
          for(std::size_t j=0; j < joint_names_.size(); ++j)
          {
            float j_error = fabs(current_joint_values[i] - metrics_db_["joint_states"][indices[0][i]][j]);
            d_j[i] = fmax(d_j[i], j_error);
          }
          // score[i] = d_q[i] + d_xyz[i] + d_j[i];
          score[i] = d_j[i];
        }
        std::size_t choice_idx = std::min_element(score.begin(), score.end()) - score.begin();
        //~ if (score[choice_idx] > 0.1)
          //~ return;
        ROS_DEBUG("nn [%d] choice [%d] score [%f] d_q [%f] d_xyz [%f] d_j [%f]", nearest_neighbors, 
                    (int)choice_idx, score[choice_idx], d_q[choice_idx], d_xyz[choice_idx], d_j[choice_idx]);
        // Populate the new joint_values
        std::ostringstream new_str;
        new_str << "new_joint_values : [";
        for(std::size_t i=0; i < joint_names_.size(); ++i)
        {
          new_joint_values.push_back(metrics_db_["joint_states"][indices[0][choice_idx]][i]);
          new_str << new_joint_values[i] << " ";            
        }
        new_str << "]";
        ik_kinematics_->setJointPositions(new_joint_values);
        // Debugging
        if (ros::Time::now() - last_motion_print_  >= ros::Duration(1.0))
        {
          last_motion_print_ = ros::Time::now();
          ROS_DEBUG_STREAM(current_str.str());
          ROS_DEBUG_STREAM("Index [" << choice_idx << "] Distance [" << dists[0][choice_idx] << "]");
          ROS_DEBUG_STREAM(new_str.str());
        }
      }
      // Command the robot to the new joint_values
      ROS_DEBUG("Joint names: %d, Joint values: %d", int(joint_names_.size()), int(new_joint_values.size()));
      double elapsed_time = (ros::Time::now() - last_ik_time_).toSec();
      last_ik_time_ = ros::Time::now();
      double velocity;
      std::string joint;
      for(std::size_t i=0; i < joint_names_.size(); ++i)
      {
        // Check the max velocity
        joint = joint_names_[i];
        if ( urdf_limits_.find(joint) == urdf_limits_.end() )
          continue;
        
        velocity = (new_joint_values[i] - current_joint_values[i])/elapsed_time;
        if (velocity > urdf_limits_[joint].max_velocity)
          new_joint_values[i] = urdf_limits_[joint].max_velocity*elapsed_time + current_joint_values[i];
        if (velocity < -urdf_limits_[joint].max_velocity)
          new_joint_values[i] = -urdf_limits_[joint].max_velocity*elapsed_time + current_joint_values[i];
        // Send the command to each joint
        std_msgs::Float64 cmd_msg;
        cmd_msg.data = new_joint_values[i];
        control_publisher_[i].publish(cmd_msg);
        ROS_DEBUG("Joint %s: %f", joint_names_[i].c_str(), new_joint_values[i]);
      }
      
    }
    
    std::string getFolderName()
    {
      std::string folder_key;
      folder_key = "f4025675e127122e084d959288e4555d/";
      std::ostringstream folder_name;
      folder_name << getenv("HOME") << "/.openrave/robot." << folder_key;
      return folder_name.str();
    }

    std::string getFilename(const std::string& database)
    {
      std::string file_key;
      file_key = ".27d697e7d8a999dfc3b0a3305edb1ee6.pp";
      std::ostringstream filename;
      filename << getFolderName() << database << file_key;
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
