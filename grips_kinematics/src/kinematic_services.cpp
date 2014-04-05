#include <ros/ros.h>

// Services
#include "grips_msgs/GetPoseMetrics.h"
#include "grips_msgs/GetStateMetrics.h"
#include "grips_msgs/GetJointLimits.h"

// Grips kinematic Interface
#include "grips_kinematics/kinematic_interface.hpp"

using namespace grips_kinematics;

class KinematicServices {
  private:
    // Ros
    ros::NodeHandle           nh_, nh_private_;
    ros::ServiceServer        ik_service_;
    ros::ServiceServer        fk_service_;
    ros::ServiceServer        limits_service_;
    // Misc
    std::string               robot_namespace_;
    std::vector<std::string>  joint_names_;
    std::string               model_frame_; 
    std::string               tip_link_;
    std::string               base_link_;
    // Kinematics
    KinematicInterfacePtr   kinematic_interface_;
    
  public:
    KinematicServices(): 
      nh_private_("~")
    { 
      // Get robot namespace
      robot_namespace_ = ros::this_node::getNamespace();
      // Kinematic interface
      kinematic_interface_.reset(new KinematicInterface());
      joint_names_ = kinematic_interface_->getActiveJointModelNames();
      model_frame_ = kinematic_interface_->getModelFrame();
      // Advertise services
      ROS_INFO("Advertising services");
      ik_service_ = nh_private_.advertiseService("get_ik_metrics", &KinematicServices::getIkMetrics, this);
      fk_service_ = nh_private_.advertiseService("get_fk_metrics", &KinematicServices::getFkMetrics, this);
      limits_service_ = nh_private_.advertiseService("get_joint_limits", &KinematicServices::getJointLimits, this);
    }
    
    bool getIkMetrics(grips_msgs::GetPoseMetrics::Request &request, grips_msgs::GetPoseMetrics::Response &response)
    {
      // Validate that frame_id = model_frame_
      if (request.header.frame_id != model_frame_)
      {
        ROS_WARN("frame_id [%s] received. Expected [%s]", request.header.frame_id.c_str(), model_frame_.c_str());
        return true;
      }
      response.found_ik = kinematic_interface_->setEndEffectorPose(request.pose, 3, 0.01);
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
      // TODO: Implement this method
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
