/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, F Suarez-Ruiz
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Francisco Suarez-Ruiz */

#include <moveit/iterative_decoupling_plugin/iterative_decoupling_plugin.h>
#include <class_loader/class_loader.h>

// KDL
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// Need a floating point tolerance when checking joint limits, in case the joint starts at limit
const double LIMIT_TOLERANCE = .0000001;

//register the plugin as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(iterative_decoupling_plugin::IterativeDecouplingPlugin, kinematics::KinematicsBase)

namespace iterative_decoupling_plugin
{

  IterativeDecouplingPlugin::IterativeDecouplingPlugin():active_(false) {}

void IterativeDecouplingPlugin::getRandomConfiguration(KDL::JntArray &jnt_array, bool lock_redundancy) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array(i) = jnt_array_vector[i];
  }
}

void IterativeDecouplingPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
                                                 const std::vector<double> &consistency_limits,
                                                 KDL::JntArray &jnt_array,
                                                 bool lock_redundancy) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0 ; i < dimension_; ++i)
    near[i] = seed_state(i);
  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), values, near, consistency_limits);
  
  for (std::size_t i = 0; i < dimension_; ++i)
    jnt_array(i) = values[i];
}

bool IterativeDecouplingPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const std::vector<double> &consistency_limits,
                                           const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool IterativeDecouplingPlugin::initialize(const std::string &robot_description,
                                     const std::string& group_name,
                                     const std::string& base_frame,
                                     const std::string& tip_frame,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  ros::NodeHandle private_handle("~");
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();
  
  // Robot dimensions
  this->d1 = 15.24e-3 + 0.33812 + 0.001;
  this->d2 = 0.52167;
  this->d3 = 0.26109;
  this->d4 = 0.13322;
  this->d5 = 0.190892;
  this->a1 = 9.82e-3;
  this->a2 = 7.5e-3; 
  this->a3 = 0.051331; 
  this->a4 = 0.787e-3;

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("idp","URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;
  
  if(!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("idp","Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if(!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("idp","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_NAMED("idp","Could not initialize tree object");
    return false;
  }

  dimension_ = joint_model_group->getVariableCount();
  ik_chain_info_.joint_names = joint_model_group->getJointModelNames();

  for (std::size_t i = 0; i < joint_model_group->getJointModels().size(); ++i)
  {
    const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group->getJointModels()[i]->getVariableBoundsMsg();
    ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
  }

  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;

  if(!joint_model_group->hasLinkModel(tip_frame_))
  {
    ROS_ERROR_NAMED("idp","Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  ik_chain_info_.link_names.push_back(tip_frame_);
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

  joint_min_.resize(this->dimension_);
  joint_max_.resize(this->dimension_);
  this->inc_max_.resize(this->dimension_);
  
  double angle_inc;
  private_handle.param("max_angle_inc", angle_inc, 0.05);

  for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_chain_info_.limits[i].min_position;
    joint_max_(i) = ik_chain_info_.limits[i].max_position;
    this->inc_max_(i) = angle_inc;
    ROS_INFO_NAMED("idp","%s:\t%f\t%f", 
        ik_chain_info_.joint_names[i+1].c_str(), joint_min_(i), joint_max_(i));
  }

  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  private_handle.param("max_solver_iterations", max_solver_iterations, 500);
  private_handle.param("epsilon", epsilon, 1e-5);
  private_handle.param(group_name+"/position_only_ik", position_ik, false);
  ROS_DEBUG_NAMED("idp","Looking in private handle: %s for param name: %s",
            private_handle.getNamespace().c_str(),
            (group_name+"/position_only_ik").c_str());

  if(position_ik)
    ROS_INFO_NAMED("idp","Using position only ik");
    
  ROS_INFO_NAMED("idp","max_solver_iterations: %d", max_solver_iterations);
  ROS_INFO_NAMED("idp","max_increment: [%.3f]", angle_inc);

  // Setup the joint state group
  state_.reset(new robot_state::RobotState(robot_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  active_ = true;
  ROS_DEBUG_NAMED("idp","KDL solver initialized");
  return true;
}

int IterativeDecouplingPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool IterativeDecouplingPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool IterativeDecouplingPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
              default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool IterativeDecouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool IterativeDecouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool IterativeDecouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool IterativeDecouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool IterativeDecouplingPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const std::vector<double> &consistency_limits,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR_NAMED("idp","kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM("Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM("Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);

  solution.resize(dimension_);

  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK2: Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " <<
                   ik_pose.orientation.y << " " <<
                   ik_pose.orientation.z << " " <<
                   ik_pose.orientation.w);
  //Do the IK
  for(unsigned int i=0; i < dimension_; i++)
    jnt_seed_state(i) = ik_seed_state[i];
  jnt_pos_in = jnt_seed_state;

  unsigned int counter(0);
  while(1)
  {
    // ROS_INFO_NAMED("idp","Iteration: %d, time: %f, Timeout: %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
    counter++;
    if(timedOut(n1,timeout))
    {
      ROS_DEBUG_NAMED("idp","IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;
    }
    int ik_valid = this->iterativeIK(jnt_pos_in, pose_desired, jnt_pos_out);
    ROS_DEBUG_NAMED("idp","IK valid: %d", ik_valid);
    if(!consistency_limits.empty())
    {
      this->getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_in, options.lock_redundant_joints);
      if( (ik_valid < 0 && !options.return_approximate_solution) || !checkConsistency(jnt_seed_state, consistency_limits, jnt_pos_out))
      {
        ROS_DEBUG_NAMED("idp","Could not find IK solution: does not match consistency limits");
        continue;
      }
    }
    else
    {
      this->getRandomConfiguration(jnt_pos_in, options.lock_redundant_joints);
      ROS_DEBUG_NAMED("idp","New random configuration");
      for(unsigned int j=0; j < dimension_; j++)
        ROS_DEBUG_NAMED("idp","%d %f", j, jnt_pos_in(j));

      if(ik_valid < 0 && !options.return_approximate_solution)
      {
        ROS_DEBUG_NAMED("idp","Could not find IK solution");
        continue;
      }
    }
    ROS_DEBUG_NAMED("idp","Found IK solution");
    for(unsigned int j=0; j < dimension_; j++)
      solution[j] = jnt_pos_out(j);
    if(!solution_callback.empty())
      solution_callback(ik_pose,solution,error_code);
    else
      error_code.val = error_code.SUCCESS;

    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("idp", "Solved after " << counter << " iterations");
      return true;
    }
  }
  ROS_DEBUG_NAMED("idp","An IK that satisifes the constraints and is collision free could not be found");
  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}

bool IterativeDecouplingPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR_NAMED("idp","kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if(joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("idp","Joint angles vector must have size: %d",dimension_);
    return false;
  }

  bool valid = true;
  Eigen::Affine3d T0_6;
  KDL::JntArray jnt_pos_in(dimension_);
  //Do the FK
  for(size_t i=0; i < dimension_; i++)
    jnt_pos_in(i) = joint_angles[i];
  this->ComputeFK_0_6(jnt_pos_in, T0_6);
  tf::poseEigenToMsg(T0_6, poses[0]);
  return valid;
}

const std::vector<std::string>& IterativeDecouplingPlugin::getJointNames() const
{
  return ik_chain_info_.joint_names;
}

const std::vector<std::string>& IterativeDecouplingPlugin::getLinkNames() const
{
  return ik_chain_info_.link_names;
}

int IterativeDecouplingPlugin::ComputeFK_0_6(const KDL::JntArray &jnt_pos_in, Eigen::Affine3d &T0_6) const
{
  double q1 = jnt_pos_in(0);
  double q2 = jnt_pos_in(1);
  double q3 = jnt_pos_in(2);
  double q4 = jnt_pos_in(3);
  double q5 = jnt_pos_in(4);
  double q6 = jnt_pos_in(5);
  T0_6.setIdentity();
  T0_6(0,0) = -cos(q6)*(-cos(q1)*cos(q5)+sin(q1+q2+q3+q4)*sin(q5)*(1.0/2.0)+sin(q1-q2-q3-q4)*sin(q5)*(1.0/2.0))+sin(q6)*(cos(q1+q2+q3+q4)*(1.0/2.0)-cos(q1-q2-q3-q4)*(1.0/2.0));
  T0_6(0,1) = -cos(q1)*sin(q5)-sin(q1+q2+q3+q4)*cos(q5)*(1.0/2.0)-sin(q1-q2-q3-q4)*cos(q5)*(1.0/2.0);
  T0_6(0,2) = cos(q6)*(cos(q1+q2+q3+q4)-cos(q1-q2-q3-q4))*(-1.0/2.0)-sin(q6)*(-cos(q1)*cos(q5)+sin(q1+q2+q3+q4)*sin(q5)*(1.0/2.0)+sin(q1-q2-q3-q4)*sin(q5)*(1.0/2.0));
  T0_6(0,3) = -a4*cos(q1)*cos(q5)+a1*cos(q2)*sin(q1)-d5*cos(q1)*sin(q5)+d2*sin(q1)*sin(q2)-d3*cos(q2)*cos(q3)*sin(q1)+a2*cos(q2)*sin(q1)*sin(q3)+a2*cos(q3)*sin(q1)*sin(q2)+d3*sin(q1)*sin(q2)*sin(q3)-d4*cos(q2)*cos(q3)*cos(q4)*sin(q1)+a3*cos(q2)*cos(q3)*sin(q1)*sin(q4)+a3*cos(q2)*cos(q4)*sin(q1)*sin(q3)+a3*cos(q3)*cos(q4)*sin(q1)*sin(q2)+d4*cos(q2)*sin(q1)*sin(q3)*sin(q4)+d4*cos(q3)*sin(q1)*sin(q2)*sin(q4)+d4*cos(q4)*sin(q1)*sin(q2)*sin(q3)-a3*sin(q1)*sin(q2)*sin(q3)*sin(q4)+a4*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)+d5*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)+d5*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)+d5*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)-a4*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)-a4*cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5)-a4*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)-d5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1);
  T0_6(1,0) = cos(q6)*(cos(q5)*sin(q1)+cos(q1+q2+q3+q4)*sin(q5)*(1.0/2.0)+cos(q1-q2-q3-q4)*sin(q5)*(1.0/2.0))+sin(q6)*(sin(q1+q2+q3+q4)*(1.0/2.0)-sin(q1-q2-q3-q4)*(1.0/2.0));
  T0_6(1,1) = -sin(q1)*sin(q5)+cos(q1+q2+q3+q4)*cos(q5)*(1.0/2.0)+cos(q1-q2-q3-q4)*cos(q5)*(1.0/2.0);
  T0_6(1,2) = cos(q6)*(sin(q1+q2+q3+q4)-sin(q1-q2-q3-q4))*(-1.0/2.0)+sin(q6)*(cos(q5)*sin(q1)+cos(q1+q2+q3+q4)*sin(q5)*(1.0/2.0)+cos(q1-q2-q3-q4)*sin(q5)*(1.0/2.0));
  T0_6(1,3) = -a1*cos(q1)*cos(q2)-a4*cos(q5)*sin(q1)-d2*cos(q1)*sin(q2)-d5*sin(q1)*sin(q5)+d3*cos(q1)*cos(q2)*cos(q3)-a2*cos(q1)*cos(q2)*sin(q3)-a2*cos(q1)*cos(q3)*sin(q2)-d3*cos(q1)*sin(q2)*sin(q3)+d4*cos(q1)*cos(q2)*cos(q3)*cos(q4)-a3*cos(q1)*cos(q2)*cos(q3)*sin(q4)-a3*cos(q1)*cos(q2)*cos(q4)*sin(q3)-a3*cos(q1)*cos(q3)*cos(q4)*sin(q2)-d4*cos(q1)*cos(q2)*sin(q3)*sin(q4)-d4*cos(q1)*cos(q3)*sin(q2)*sin(q4)-d4*cos(q1)*cos(q4)*sin(q2)*sin(q3)+a3*cos(q1)*sin(q2)*sin(q3)*sin(q4)-d5*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)-d5*cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)-d5*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)+a4*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)+a4*cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5)+a4*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)+d5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)-a4*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5);
  T0_6(2,0) = sin(q2+q3+q4+q6)*(-1.0/2.0)+sin(q2+q3+q4-q6)*(1.0/2.0)+sin(q2+q3+q4)*cos(q6)*sin(q5);
  T0_6(2,1) = sin(q2+q3+q4+q5)*(1.0/2.0)+sin(q2+q3+q4-q5)*(1.0/2.0);
  T0_6(2,2) = cos(q2+q3+q4+q6)*(1.0/2.0)+cos(q2+q3+q4-q6)*(1.0/2.0)+sin(q2+q3+q4)*sin(q5)*sin(q6);
  T0_6(2,3) = d1+a4*cos(q2+q3+q4+q5)*(1.0/2.0)+d5*sin(q2+q3+q4+q5)*(1.0/2.0)+a2*cos(q2+q3)+d3*sin(q2+q3)+d2*cos(q2)-a1*sin(q2)-a4*cos(q2+q3+q4-q5)*(1.0/2.0)+d5*sin(q2+q3+q4-q5)*(1.0/2.0)+a3*cos(q2+q3+q4)+d4*sin(q2+q3+q4);
  T0_6(3,3) = 1.0;
  return 1;
}

int IterativeDecouplingPlugin::ComputeFK_0_3(const KDL::JntArray &jnt_pos_in, Eigen::Affine3d &T0_3) const
{
  double q1 = jnt_pos_in(0);
  double q2 = jnt_pos_in(1);
  double q3 = jnt_pos_in(2);
  T0_3.setIdentity();
  T0_3(0,0) = cos(q1);
  T0_3(0,1) = -cos(q2+q3)*sin(q1);
  T0_3(0,2) = sin(q2+q3)*sin(q1);
  T0_3(0,3) = sin(q1)*(-d3*cos(q2+q3)+a2*sin(q2+q3)+a1*cos(q2)+d2*sin(q2));
  T0_3(1,0) = sin(q1);
  T0_3(1,1) = cos(q2+q3)*cos(q1);
  T0_3(1,2) = -sin(q2+q3)*cos(q1);
  T0_3(1,3) = -cos(q1)*(-d3*cos(q2+q3)+a2*sin(q2+q3)+a1*cos(q2)+d2*sin(q2));
  T0_3(2,1) = sin(q2+q3);
  T0_3(2,2) = cos(q2+q3);
  T0_3(2,3) = d1+a2*cos(q2+q3)+d3*sin(q2+q3)+d2*cos(q2)-a1*sin(q2);
  T0_3(3,3) = 1.0;
  return 1;
}

int IterativeDecouplingPlugin::ComputeFK_3_6(const KDL::JntArray &jnt_pos_in, Eigen::Affine3d &T3_6) const
{
  double q1 = jnt_pos_in(3);
  double q2 = jnt_pos_in(4);
  double q3 = jnt_pos_in(5);
  T3_6.setIdentity();
  T3_6(0,0) = cos(q2)*cos(q3);
  T3_6(0,1) = -sin(q2);
  T3_6(0,2) = cos(q2)*sin(q3);
  T3_6(0,3) = -a4*cos(q2)-d5*sin(q2);
  T3_6(1,0) = sin(q1)*sin(q3)+cos(q1)*cos(q3)*sin(q2);
  T3_6(1,1) = cos(q1)*cos(q2);
  T3_6(1,2) = -cos(q3)*sin(q1)+cos(q1)*sin(q2)*sin(q3);
  T3_6(1,3) = d4*cos(q1)-a3*sin(q1)+d5*cos(q1)*cos(q2)-a4*cos(q1)*sin(q2);
  T3_6(2,0) = -cos(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2);
  T3_6(2,1) = cos(q2)*sin(q1);
  T3_6(2,2) = cos(q1)*cos(q3)+sin(q1)*sin(q2)*sin(q3);
  T3_6(2,3) = a3*cos(q1)+d4*sin(q1)+d5*cos(q2)*sin(q1)-a4*sin(q1)*sin(q2);
  T3_6(3,3) = 1.0;
  return 1;
}

bool IterativeDecouplingPlugin::validSolution(Eigen::Affine3d Tsol, Eigen::Affine3d Tgoal) const
{
  KDL::Frame f, p_in;
  tf::transformEigenToKDL(Tsol, f);
  tf::transformEigenToKDL(Tgoal, p_in);
  return (KDL::Equal(KDL::diff(f,p_in), KDL::Twist::Zero(), epsilon_));
}

double IterativeDecouplingPlugin::harmonize(KDL::JntArray &q_old, KDL::JntArray &q_new) const
{
  double delta, max_delta = DBL_MIN;
  for(size_t i=0; i< this->dimension_; ++i)
  {
    // Harmonize
    while(q_old(i) > 2*M_PI)
      q_old(i) -= 2*M_PI;
    while(q_old(i) < -2*M_PI)
      q_old(i) += 2*M_PI;
    while(q_new(i) > 2*M_PI)
      q_new(i) -= 2*M_PI;
    while(q_new(i) < -2*M_PI)
      q_new(i) += 2*M_PI;
    
    // Force to be within the joint limits
    /*if(q_new(i) < (this->joint_min_(i)-LIMIT_TOLERANCE))
      q_new(i) = this->joint_min_(i);
    
    if(q_new(i) > (this->joint_max_(i)+LIMIT_TOLERANCE))
      q_new(i) = this->joint_max_(i);*/
      
    delta = fabs(q_old(i) - q_new(i));
    if (delta > max_delta)
      max_delta = delta;
  }
  return max_delta;
}

int IterativeDecouplingPlugin::iterativeIK(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) const
{
  int iter = 0, ik_iterations = -3;
  KDL::JntArray q_old(dimension_), q_new(dimension_);
  KDL::JntArray inc_new(dimension_), inc_old(dimension_), inc_max = this->inc_max_;
  q_old = q_init;
  double alpha, beta, gamma, q2_0, q3_0, C3, S3, S5, px, py, pz, L2, L3, py_sign;
  double error, pos_error, rot_error;
  Eigen::Affine3d Tg0_6, Tc0_3, Tc0_6, Tc3_6, Tn0_3, Tsol;
  Eigen::Matrix3d Re4_6;
  Eigen::Vector3d Pe0_3;
  // Populate the Goal Transformation 
  tf::transformKDLToEigen(p_in, Tg0_6);
  ROS_DEBUG_STREAM_NAMED("idp","Tg0_6:\n" << Tg0_6.matrix());
  // Initial values of q2, q3
  q2_0 = M_PI_2 + atan2(a1,d2);
  q3_0 = atan2(a2,d3) + atan2(d2, a1);
  while (iter < this->max_solver_iterations_)
  {
    // Estimate the middle point Pe0_3
    this->ComputeFK_0_3(q_old, Tc0_3);
    this->ComputeFK_3_6(q_old, Tc3_6);
    Pe0_3 = Tg0_6.translation() - Tc0_3.rotation() * Tc3_6.translation();
    ROS_DEBUG_STREAM_NAMED("idp","Pe0_3:\n" << Pe0_3);
    // Calculate the new q1, q2, q3
    px = Pe0_3(0); py = Pe0_3(1); pz = Pe0_3(2);
    L2 = sqrt(pow(d2,2) + pow(a1,2));
    L3 = sqrt(pow(d3,2) + pow(a2,2));
    q_new(0) = -atan2(px, py);
    C3 = (pow(L2,2)+pow(L3,2)-pow(px,2)-pow(py,2)-pow(pz - d1,2)) / (2*L2*L3);
    S3 = sqrt(1-pow(C3,2));
    if (fabs(C3) <= 1)
    {
      // Due to the range of q3, gamma will always be within the 1rst and 2nd quadrants
      gamma = atan2(S3, C3);
      py_sign = (py > 0.0) ? 1.0 : ((py < 0.0) ? -1.0 : 0.0);
      beta = atan2(pz-d1, py_sign * sqrt(pow(px,2)+pow(py,2)));
      alpha = atan2(L3*sin(gamma), L2 - L3*cos(gamma));
      q_new(1) = beta + alpha - q2_0;
      q_new(2) = gamma - q3_0;
      // Regulate the delta at each iteration (improves convergency)
      for (size_t i = 0; i < 3; i++)
      {
        inc_new(i) = q_new(i) - q_old(i);
        if (inc_new(i) > inc_max(i))
          inc_new(i) = inc_max(i);
        if (inc_new(i) < -inc_max(i))
          inc_new(i) = -inc_max(i);
        if (inc_new(i) == -inc_old(i))
          inc_max(i) /= 2;
        inc_old(i) = inc_new(i);
        q_new(i) = q_old(i) + inc_new(i);
      }
    }
    ROS_DEBUG_NAMED("idp","[px, py, pz]: [%f, %f, %f]", px, py, pz);
    ROS_DEBUG_NAMED("idp","C3: %f, S3: %f, ", C3, S3);
    // Calculate the new middle point Tn0_3
    this->ComputeFK_0_3(q_new, Tn0_3);
    ROS_DEBUG_STREAM_NAMED("idp","Tn0_3:\n" << Tn0_3.matrix());
    // TODO: This operation could be faster using quaternions
    // Estimate the orientation R4_6
    Re4_6 = Tn0_3.rotation().inverse() * Tg0_6.rotation();
    ROS_DEBUG_STREAM_NAMED("idp","Re4_6:\n" << Re4_6);
    // Calculate the new q4, q5, q6
    q_new(3) = atan2(Re4_6(2,1), Re4_6(1,1));
    /* S5 = -Re4_6(0,1);
    q_new(4) = atan2(S5, sqrt(1-pow(S5,2)));*/
    q_new(4) = asin(-Re4_6(0,1));
    q_new(5) = atan2(Re4_6(0,2), Re4_6(0,0));
    
    // Regulate the delta at each iteration (improves convergency)
    for (size_t i = 3; i < this->dimension_; i++)
    {
      inc_new(i) = q_new(i) - q_old(i);
      if (inc_new(i) > inc_max(i))
        inc_new(i) = inc_max(i);
      if (inc_new(i) < -inc_max(i))
        inc_new(i) = -inc_max(i);
      if (inc_new(i) == -inc_old(i))
        inc_max(i) /= 2;
      inc_old(i) = inc_new(i);
      q_new(i) = q_old(i) + inc_new(i);
    }
    
    error = harmonize(q_old, q_new);
    ROS_DEBUG_NAMED("idp","q_new: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", q_new(0), q_new(1), q_new(2), q_new(3), q_new(4), q_new(5));
    ROS_DEBUG_NAMED("idp","q_old: [%f, %f, %f, %f, %f, %f]", q_old(0), q_old(1), q_old(2), q_old(3), q_old(4), q_old(5));
    ROS_DEBUG_NAMED("idp", "Error: %f", error);
    iter++;
    this->ComputeFK_0_6(q_new, Tsol);
    bool ik_found = this->validSolution(Tsol, Tg0_6);
    
    if (ik_found)
    {
      if(obeysLimits(q_new))
        ik_iterations = iter;
      break;
    }
    q_old = q_new;
  }
  q_out = q_new;
  return ik_iterations;
}

bool IterativeDecouplingPlugin::obeysLimits(const KDL::JntArray &jnt_pos) const
{
  bool obeys_limits = true;
  for(size_t i = 0; i < this->dimension_; i++)
  {
    if( (jnt_pos(i) < (this->joint_min_(i)-LIMIT_TOLERANCE)) || (jnt_pos(i) > (this->joint_max_(i)+LIMIT_TOLERANCE)) )
    {
      // One element of solution is not within limits
      obeys_limits = false;
      ROS_DEBUG_STREAM_NAMED("idp","Not in limits! " << i << " value " << jnt_pos(i) << " has limit being  " << joint_min_(i) << " to " << joint_max_(i));
      break;
    }
  }
  return obeys_limits;
}

} // namespace
