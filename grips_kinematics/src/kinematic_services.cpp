#include <ros/ros.h>

// Services
#include <grips_msgs/GetPoseMetrics.h>
#include <grips_msgs/GetStateMetrics.h>
#include <grips_msgs/GetJointLimits.h>

// Dynamic Reconfiguration
#include <dynamic_reconfigure/server.h>
#include <grips_msgs/KinematicsSolversConfig.h>

// Grips kinematic Interface
#include <grips_kinematics/kinematic_interface.hpp>

using namespace grips_kinematics;

class KinematicServices {
  private:
    // Ros
    ros::NodeHandle           nh_, nh_private_;
    ros::ServiceServer        ik_service_;
    ros::ServiceServer        fk_service_;
    ros::ServiceServer        limits_service_;
    // dynamic_reconfigure
    dynamic_reconfigure::Server<grips_msgs::KinematicsSolversConfig>                server_;
    dynamic_reconfigure::Server<grips_msgs::KinematicsSolversConfig>::CallbackType  dyn_callback_;
    // Misc
    std::string               robot_namespace_;
    std::string               planning_group_;
    std::vector<std::string>  joint_names_;
    std::string               model_frame_; 
    std::string               tip_link_;
    std::string               base_link_;
    bool                      first_call_dyn_;
    bool                      dyn_reconfig_enabled_;
    // Kinematics
    KinematicInterfacePtr     kinematic_interface_;
    std::map<std::string, joint_limits_interface::JointLimits> urdf_limits_;
    
  public:
    KinematicServices(): 
      nh_private_("~")
    { 
      // Get robot namespace
      robot_namespace_ = ros::this_node::getNamespace();
      // Get parameters from the server
      nh_private_.param(std::string("planning_group"), planning_group_, std::string("arm"));
      if (!nh_private_.hasParam("planning_group"))
        ROS_WARN_STREAM("Parameter [~planning_group] not found, using default: " << planning_group_);
      nh_private_.param(std::string("ik_dynamic_reconfigure"), dyn_reconfig_enabled_, true);
      if (!nh_private_.hasParam("ik_dynamic_reconfigure"))
        ROS_WARN_STREAM("Parameter [~ik_dynamic_reconfigure] not found, using default: " << dyn_reconfig_enabled_);
      // Kinematic interface
      kinematic_interface_.reset(new KinematicInterface());
      joint_names_ = kinematic_interface_->getActiveJointModelNames();
      model_frame_ = kinematic_interface_->getModelFrame();
      urdf_limits_ = kinematic_interface_->getJointLimits();
      // Advertise services
      ROS_INFO("Advertising services");
      ik_service_ = nh_private_.advertiseService("get_ik_metrics", &KinematicServices::getIkMetrics, this);
      fk_service_ = nh_private_.advertiseService("get_fk_metrics", &KinematicServices::getFkMetrics, this);
      limits_service_ = nh_private_.advertiseService("get_joint_limits", &KinematicServices::getJointLimits, this);
      // Setup dynamic_reconfigure Server
      first_call_dyn_ = true;
      dyn_callback_ = boost::bind(&KinematicServices::dynamicReconfiguration, this, _1, _2);
      server_.setCallback(dyn_callback_);
    }
    
    void dynamicReconfiguration(grips_msgs::KinematicsSolversConfig &config, uint32_t level) 
    {
      if (!dyn_reconfig_enabled_)
        return;
      
      // Avoid the initialization execution
      if (first_call_dyn_)
      {
        first_call_dyn_ = false;
        return;
      }
      // Max angle increment parameter (IterativeDecouplingPlugin only)
      nh_private_.setParam("max_angle_inc", config.max_increment);
      
      // Kinematic solver parameter
      std::string selected_solver, current_solver;
      ros::param::get(ros::this_node::getName() + "/" + planning_group_ + "/kinematics_solver", current_solver);
      selected_solver = current_solver;
      switch (config.kinematics_solver)
      {
        case 0: selected_solver = "grips_arm_kinematics/IKFastTransform6dPlugin"; break;
        case 1: selected_solver = "lma_kinematics_plugin/LMAKinematicsPlugin"; break;
        case 2: selected_solver = "grips_arm_kinematics/IterativeDecouplingPlugin"; break;
        case 3: selected_solver = "kdl_kinematics_plugin/KDLKinematicsPlugin"; break;
      }
      
      // Reset the Kinematic interface to load the selected plugin
      ros::param::set(ros::this_node::getName() + "/" + planning_group_ + "/kinematics_solver", selected_solver);
      kinematic_interface_.reset(new KinematicInterface());
    }
    
    bool getIkMetrics(grips_msgs::GetPoseMetrics::Request &request, grips_msgs::GetPoseMetrics::Response &response)
    {
      // Validate that frame_id = model_frame_
      Eigen::Affine3d T_frame_id;
      if (request.header.frame_id != model_frame_)
      {
        // TODO: Validate that the is within model_frame_ and tip_link
        T_frame_id = kinematic_interface_->getGlobalLinkTransform(request.header.frame_id);
        //~ ROS_WARN("frame_id [%s] received. Expected [%s]", request.header.frame_id.c_str(), model_frame_.c_str());
        //~ return true;
      }
      else
        T_frame_id.setIdentity();
      Eigen::Affine3d T_goal;
      tf::poseMsgToEigen(request.pose, T_goal);
      //~ ROS_INFO_STREAM("\nRequest pose:\n" << T_goal);
      //~ ROS_INFO_STREAM("\nT_frame_id:\n" << T_frame_id.matrix());
      T_goal = T_frame_id * T_goal;
      //~ ROS_INFO_STREAM("\nT_goal:\n" << T_goal.matrix());
      ros::Time begin = ros::Time::now();
      response.found_ik = kinematic_interface_->setEndEffectorPose(request.pose);
      response.duration = ros::Time::now() - begin;
      // If IK not found return with default values
      if (!response.found_ik)
        return true;
      
      // Populate the joint_states
      response.joint_states.name.resize(joint_names_.size());
      response.joint_states.position.resize(joint_names_.size());
      std::vector<double> joint_values;
      kinematic_interface_->getJointPositions(joint_values);
      for(std::size_t i = 0; i < joint_names_.size(); ++i)
      {
        response.joint_states.name[i] = joint_names_[i];
        response.joint_states.position[i] = joint_values[i];
      }
      //~ // Populate the estimated pose
      //~ kinematic_interface_->setJointPositions(joint_values);
      //~ Eigen::Affine3d T_estimated;
      //~ T_estimated = T_frame_id.inverse() * kinematic_interface_->getEndEffectorTransform();
      //~ ROS_DEBUG_STREAM("\n" << T_estimated.matrix());
      //~ tf::poseEigenToMsg(T_estimated, response.estimated_pose);
      //~ 
      //~ // Populate the error pose
      //~ tf::poseEigenToMsg(T_goal.inverse() * T_estimated, response.estimation_error);
      
      response.found_group = true;
      // TODO: Calculate the metrics
      response.manipulability_index = 1.0;
      response.manipulability = 1.0;
    }
    
    bool getFkMetrics(grips_msgs::GetStateMetrics::Request &request, grips_msgs::GetStateMetrics::Response &response)
    {
      // Validate that frame_id = model_frame_
      if (request.joint_states.header.frame_id != model_frame_)
      {
        ROS_WARN("frame_id [%s] received. Expected [%s]", request.joint_states.header.frame_id.c_str(), model_frame_.c_str());
        response.found_group = false;
        return true;
      }
      // Populate the joint_values
      bool changed = false;
      std::vector<double> joint_values(joint_names_.size());
      for(std::size_t i=0; i < joint_names_.size(); ++i)
      {
        for(std::size_t j=0; j < request.joint_states.position.size(); ++j)
        {
          // If at least 1 joint state has been send we accept this request
          if (joint_names_[i] == request.joint_states.name[j])
          {
            changed = true;
            joint_values[i] = request.joint_states.position[j];
            break;
          }
        }
        ROS_DEBUG("Joint [%s]: %f", joint_names_[i].c_str(), joint_values[i]);
      }
      if (!changed)
        ROS_WARN("Unknown [joint_states.name] values");
      // Do FK
      kinematic_interface_->setJointPositions(joint_values);
      Eigen::Affine3d T_end;
      T_end = kinematic_interface_->getEndEffectorTransform();
      ROS_DEBUG_STREAM("\n" << T_end.matrix());
      tf::poseEigenToMsg(T_end, response.pose);
      response.found_group = true;
      
      // Calculate the metrics
      double manipulability_index, manipulability;
      if (kinematic_interface_->getManipulabilityIndex(manipulability_index) && kinematic_interface_->getManipulability(manipulability))
      {
        response.manipulability_index = manipulability_index;
        response.manipulability = manipulability;
      }
      else
        response.found_group = false;
        
      return true;
    }
    
    bool getJointLimits(grips_msgs::GetJointLimits::Request &request, grips_msgs::GetJointLimits::Response &response)
    {
      std::string joint;
      for(int i = 0; i < request.name.size(); ++i)
      {
        joint = request.name[i];
        if ( urdf_limits_.find(joint) == urdf_limits_.end() )
          ROS_WARN("Joint [%s] not found in the urdf", request.name[i].c_str());
        else {
          response.name.push_back(joint);
          response.min_position.push_back(urdf_limits_[joint].min_position);
          response.max_position.push_back(urdf_limits_[joint].max_position);
          response.max_velocity.push_back(urdf_limits_[joint].max_velocity);
        }
      }
      return true;
    }
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "kinematic_services");
  KinematicServices server;
  ros::spin();
  ros::shutdown();
  return 0;
}
