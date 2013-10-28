#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <eigen_conversions/eigen_msg.h>

// Services
#include "grips_msgs/GetPoseMetrics.h"
#include "grips_msgs/GetStateMetrics.h"
#include "grips_msgs/GetJointLimits.h"

// Joint limits
#include <joint_limits_interface/joint_limits_urdf.h>

class KinematicServices {
	private:
		// Ros
		ros::NodeHandle 													nh, nh_private;
		ros::ServiceServer 												ik_service;
		ros::ServiceServer 												fk_service;
		ros::ServiceServer 												limits_service;
		// Kinematics
		robot_model_loader::RobotModelLoader			rm_loader;
		robot_model::RobotModelPtr 								kinematic_model;
		robot_state::RobotStatePtr 								kinematic_state;
		robot_state::JointModelGroup* 						joint_model_group;
		// Joint limits
		std::map<std::string, joint_limits_interface::JointLimits> urdf_limits;
		//Metric calculations
		kinematics_metrics::KinematicsMetricsPtr 	kinematics_metrics;
		// Misc
		std::vector<std::string> 									joint_names;
		std::string 															robot_namespace;
		std::string 															planning_group; 
		std::string 															model_frame; 
		std::string																tip_link;
		std::string																base_link;
		
	public:
		KinematicServices(): 
			nh_private ("~"), 
			// TODO: The robot description shouldn't be hard coded
			rm_loader("/grips/robot_description"),
			joint_model_group(0)
		{ 
			// Get parameters from the server
			if (!nh_private.hasParam("planning_group")) {
				ros::param::param(std::string("~planning_group"), this->planning_group, std::string("arm"));
				ROS_WARN("Parameter [~planning_group] not found, using default: arm");
			}
			// Get robot namespace
			this->robot_namespace = ros::this_node::getNamespace();
			if (this->robot_namespace.rfind("/") != this->robot_namespace.length()-1)
				this->robot_namespace.append("/");
			if (this->robot_namespace.length() > 1) {
				if (this->robot_namespace[0] == this->robot_namespace[1])
					this->robot_namespace.erase(0, 1);
			}
			std::string kinematic_solver;
			if (ros::param::get(this->robot_namespace + "/kinematic_srv/arm/kinematics_solver", kinematic_solver))
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
			// TODO: Here should read ONLY controllable joints (anchor is been added)
			for(std::size_t i = 1; i < tmp.size(); ++i) {
				this->joint_names.push_back(tmp[i]);
				ROS_DEBUG("Joint [%d]: %s", int(i), this->joint_names.back().c_str());
			}
			
			// Setup the kinematic metrics
			this->kinematics_metrics.reset(new kinematics_metrics::KinematicsMetrics(this->kinematic_model));
			/* Multiplier for JointLimitsPenalty.  Set penalty_multiplier to 0
			if you don't want this to have any effect on the metrics*/
			this->kinematics_metrics->setPenaltyMultiplier(0.0);
			
			// Advertise services
			ROS_INFO("Advertising services");
			this->ik_service = nh_private.advertiseService("get_ik_metrics", &KinematicServices::get_ik_metrics, this);
			this->fk_service = nh_private.advertiseService("get_fk_metrics", &KinematicServices::get_fk_metrics, this);
			this->limits_service = nh_private.advertiseService("get_joint_limits", &KinematicServices::get_joint_limits, this);
			
			// Get the joint limits
			typedef std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it_type;
			for(it_type it = this->rm_loader.getURDF()->joints_.begin(); it != this->rm_loader.getURDF()->joints_.end(); it++)
			{
				joint_limits_interface::getJointLimits(it->second, this->urdf_limits[it->first]);
				ROS_DEBUG_STREAM("Joint " << it->first << ": " << this->urdf_limits[it->first].min_position << " " << this->urdf_limits[it->first].max_position);
			}			
		}
		
		bool get_ik_metrics(grips_msgs::GetPoseMetrics::Request &request, grips_msgs::GetPoseMetrics::Response &response)
		{
			// TODO: Validate that frame_id = this->model_frame
			// TODO: Validate that link_name = this->tip_link
			Eigen::Affine3d T_base, T_end, T;
			tf::poseMsgToEigen(request.pose, T_end);
			T_base = this->kinematic_state->getGlobalLinkTransform(this->base_link);
			T = T_base * T_end;
			// Here 3 is the number of random restart and 0.005 is the allowed time after each restart
			response.found_ik = this->kinematic_state->setFromIK(this->joint_model_group, T_end, 3, 0.01);
			
			// If IK not found return with default values
			if (!response.found_ik)
				return true;
			
			// Populate the joint_states
			response.joint_states.name.resize(this->joint_names.size());
			response.joint_states.position.resize(this->joint_names.size());
			std::vector<double> joint_values;
			this->kinematic_state->copyJointGroupPositions(this->joint_model_group, joint_values);
			for(std::size_t i = 0; i < this->joint_names.size(); ++i)
			{
				response.joint_states.name[i] = this->joint_names[i];
				response.joint_states.position[i] = joint_values[i];
			}
			
			response.found_group = true;
			/* Calculate the metrics
			double manipulability_index, manipulability;
			if (this->kinematics_metrics->getManipulabilityIndex(*this->kinematic_state, this->planning_group, manipulability_index))
				response.manipulability_index = manipulability_index;
			else
				response.found_group = false;
			if (this->kinematics_metrics->getManipulability(*this->kinematic_state, this->planning_group, manipulability))
				response.manipulability = manipulability;
			else
				response.found_group = false;*/
			
			return true;
		}
		
		bool get_fk_metrics(grips_msgs::GetStateMetrics::Request &request, grips_msgs::GetStateMetrics::Response &response)
		{
			// Validate that frame_id = this->model_frame
			if (request.joint_states.header.frame_id != this->model_frame)
			{
				ROS_WARN("frame_id [%s] received. Expected [%s]", request.joint_states.header.frame_id.c_str(), this->model_frame.c_str());
				response.found_group = false;
				return true;
			}
			// Populate the joint_values
			bool changed = false;
			std::vector<double> joint_values(this->joint_names.size());
			for(std::size_t i=0; i < this->joint_names.size(); ++i)
			{
				for(std::size_t j=0; j < request.joint_states.position.size(); ++j)
				{
					// If at least 1 joint state has been send we accept this request
					if (this->joint_names[i] == request.joint_states.name[j])
					{
						changed = true;
						joint_values[i] = request.joint_states.position[j];
						break;
					}
				}
				ROS_DEBUG("Joint [%s]: %f", this->joint_names[i].c_str(), joint_values[i]);
			}
			if (!changed)
				ROS_WARN("Unknown [joint_states.name] values");
			// Do FK
			this->kinematic_state->setJointGroupPositions(this->joint_model_group, joint_values);
			Eigen::Affine3d T_end;
			T_end = this->kinematic_state->getGlobalLinkTransform(this->tip_link);
			ROS_DEBUG_STREAM("\n" << T_end.matrix());
			tf::poseEigenToMsg(T_end, response.pose);
			response.found_group = true;
			/* Calculate the metrics
			double manipulability_index, manipulability;
			if (this->kinematics_metrics->getManipulabilityIndex(*this->kinematic_state, this->planning_group, manipulability_index))
				response.manipulability_index = manipulability_index;
			else
				response.found_group = false;
			if (this->kinematics_metrics->getManipulability(*this->kinematic_state, this->planning_group, manipulability))
				response.manipulability = manipulability;
			else
				response.found_group = false;*/
			return true;
		}
		
		bool get_joint_limits(grips_msgs::GetJointLimits::Request &request, grips_msgs::GetJointLimits::Response &response)
		{
			std::string joint;
			for(int i = 0; i < request.name.size(); ++i)
			{
				joint = request.name[i];
				if ( this->urdf_limits.find(joint) == this->urdf_limits.end() )
					ROS_WARN("Joint [%s] not found in the urdf", request.name[i].c_str());
				else {
					response.name.push_back(joint);
					response.min_position.push_back(this->urdf_limits[joint].min_position);
					response.max_position.push_back(this->urdf_limits[joint].max_position);
					response.max_velocity.push_back(this->urdf_limits[joint].max_velocity);
				}
			}
			return true;			
		}
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "grips_kinematic_services");
  KinematicServices k_srv;
  ros::spin();
  ros::shutdown();
  return 0;
}
