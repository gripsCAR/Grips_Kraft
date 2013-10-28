/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/**
 *  \author Francisco Suarez Ruiz
 *  \desc   Plugin for the dexterity test
 */
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

using std::endl;

namespace gazebo
{
class DexterityTest : public ModelPlugin
{
private:
	physics::WorldPtr 			world_;
	physics::ModelPtr 			model_;
	physics::Joint_V				joints_;
  physics::Link_V 				links_;
	event::ConnectionPtr 		update_connection_;
	common::Time						last_time_;
	// Poses of the anchors
	std::vector<math::Pose>	wall_anchor_;
	std::vector<math::Pose>	back_anchor_;
	std::vector<math::Pose>	shaft_anchor_;

	/* Constructor */
  public: DexterityTest() : ModelPlugin() {
  }
  
  /* Destructor */
  public: virtual ~DexterityTest()
  {
		event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  }
	
	/* Load the plugin */
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
		int i;
		// Store the pointer to the model and world
		this->model_ = _parent;
		this->world_ = model_->GetWorld();
		// Get joints and links
		this->joints_ = this->model_->GetJoints();
		this->links_ = this->model_->GetLinks();
		/* Debug msg joints and links names
		for (i = 0; i < this->joints_.size(); i++)
			gzdbg << "Joint " << i << ": " << this->joints_[i]->GetName() << endl;
		for (i = 0; i < this->links_.size(); i++)
			gzdbg << "Link " << i << ": " << this->links_[i]->GetName() << endl;*/
		// Load parameters
		if (!_sdf->HasElement("wall_anchor_1")) {
			gzerr << "dexterity_test_plugin missing <wall_anchor_1>" << endl;
			return; }
		else
			this->wall_anchor_.push_back(_sdf->GetElement("wall_anchor_1")->Get<math::Pose>());
		if (!_sdf->HasElement("wall_anchor_2")) {
			gzerr << "dexterity_test_plugin missing <wall_anchor_2>" << endl;
			return; }
		else
			this->wall_anchor_.push_back(_sdf->GetElement("wall_anchor_2")->Get<math::Pose>());
		if (!_sdf->HasElement("wall_anchor_3")) {
			gzerr << "dexterity_test_plugin missing <wall_anchor_3>" << endl;
			return; }
		else
			this->wall_anchor_.push_back(_sdf->GetElement("wall_anchor_3")->Get<math::Pose>());
		if (!_sdf->HasElement("wall_anchor_4")) {
			gzerr << "dexterity_test_plugin missing <wall_anchor_4>" << endl;
			return; }
		else
			this->wall_anchor_.push_back(_sdf->GetElement("wall_anchor_4")->Get<math::Pose>());
		if (!_sdf->HasElement("back_anchor_1")) {
			gzerr << "dexterity_test_plugin missing <back_anchor_1>" << endl;
			return; }
		else
			this->back_anchor_.push_back(_sdf->GetElement("back_anchor_1")->Get<math::Pose>());
		if (!_sdf->HasElement("back_anchor_2")) {
			gzerr << "dexterity_test_plugin missing <back_anchor_2>" << endl;
			return; }
		else
			this->back_anchor_.push_back(_sdf->GetElement("back_anchor_2")->Get<math::Pose>());
		if (!_sdf->HasElement("shaft_anchor_1")) {
			gzerr << "dexterity_test_plugin missing <shaft_anchor_1>" << endl;
			return; }
		else
			this->shaft_anchor_.push_back(_sdf->GetElement("shaft_anchor_1")->Get<math::Pose>());
		if (!_sdf->HasElement("shaft_anchor_2")) {
			gzerr << "dexterity_test_plugin missing <shaft_anchor_2>" << endl;
			return; }
		else
			this->shaft_anchor_.push_back(_sdf->GetElement("shaft_anchor_2")->Get<math::Pose>());
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)" << endl;
      return;
    }
    // Reset Time
		this->last_time_ = this->world_->GetSimTime();
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DexterityTest::UpdateChild, this));
    this->BlueMsg("dexterity_test_plugin successfully loaded!");
  }
  
  /* Update the plugin */
  private: void UpdateChild()
  {
		common::Time current_time = this->world_->GetSimTime();
		if (current_time > this->last_time_)
		{
			this->CheckJointCreation();
		}
	}
	
	/* Check the joints creation and removal */
	bool CheckJointCreation()
	{
		int min_idx, joint_idx;
		double position;
		std::vector<double> pos_error, rot_error;
		pos_error.resize(this->wall_anchor_.size());
		rot_error.resize(this->wall_anchor_.size());
		// left joint
		/* For debugging you can push the link up
		this->links_[2]->SetLinearVel(math::Vector3(0, 0, 0.1)); */
		joint_idx = 2;
		if (!this->joints_[joint_idx])
		{
			min_idx = this->ClosestAnchor(joint_idx, pos_error, rot_error, this->wall_anchor_);
			// Tolerance: position in meters, rotation in ¿radians?
			if (pos_error[min_idx] < 0.0025 && rot_error[min_idx] < 0.01)
			{
				this->joints_[joint_idx] = this->AddJoint(this->world_, this->model_,
																				this->links_[0], this->links_[joint_idx],
																				"prismatic",
																				math::Pose(0, 0, 0, 0, 0, 0),
																				math::Vector3(0, 0, 1),
																				0.015, -0.04);
			}
		}
		else
		{
			// check joint position to disconnect
			position = this->joints_[joint_idx]->GetAngle(0).Radian();
			if (position > 0.01)
				this->RemoveJoint(this->joints_[joint_idx]);
		}
		// right joint
		joint_idx = 3;
		if (!this->joints_[joint_idx])
		{
			min_idx = this->ClosestAnchor(joint_idx, pos_error, rot_error, this->wall_anchor_);
			// Tolerance: position in meters, rotation in ¿radians?
			if (pos_error[min_idx] < 0.0025 && rot_error[min_idx] < 0.01)
			{
				this->joints_[joint_idx] = this->AddJoint(this->world_, this->model_,
																				this->links_[0], this->links_[joint_idx],
																				"prismatic",
																				math::Pose(0, 0, 0, 0, 0, 0),
																				math::Vector3(0, 0, 1),
																				0.015, -0.04);
			}
		}
		else
		{
			// check joint position to disconnect
			position = this->joints_[joint_idx]->GetAngle(0).Radian();
			if (position > 0.01)
				this->RemoveJoint(this->joints_[joint_idx]);
		}
		// back joint
		joint_idx = 4;
		if (!this->joints_[joint_idx])
		{
			min_idx = this->ClosestAnchor(joint_idx, pos_error, rot_error, this->wall_anchor_);
			// Tolerance: position in meters, rotation in ¿radians?
			if (pos_error[min_idx] < 0.0025 && rot_error[min_idx] < 0.01)
			{
				this->joints_[joint_idx] = this->AddJoint(this->world_, this->model_,
																				this->links_[0], this->links_[joint_idx],
																				"prismatic",
																				math::Pose(0, 0, 0, 0, 0, 0),
																				math::Vector3(0, 0, 1),
																				0.015, -0.024);
			}
		}
		else
		{
			// check joint position to disconnect
			position = this->joints_[joint_idx]->GetAngle(0).Radian();
			if (position > 0.01)
				this->RemoveJoint(this->joints_[joint_idx]);
		}
		return true;
	}
	
	/* Dynamically add joint between 2 links */
	private: physics::JointPtr AddJoint(physics::WorldPtr _world,
																								physics::ModelPtr _model,
																								physics::LinkPtr _parent,
																								physics::LinkPtr _child,
																								std::string _type,
																								math::Pose _pose,
																								math::Vector3 _axis,
																								double _upper, double _lower)
	{
		physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
			_type, _model);
		joint->Attach(_parent, _child);
		// load adds the joint to a vector of shared pointers kept
		// in parent and child links, preventing joint from being destroyed.
		joint->Load(_parent, _child, _pose);
		joint->SetAxis(0, _axis);
		joint->SetHighStop(0, _upper);
		joint->SetLowStop(0, _lower);
		joint->SetName(_child->GetName() + "_joint");
		joint->Init();
		// disable collision between the link pair
		_parent->SetCollideMode("fixed");
		_child->SetCollideMode("fixed");
		return joint;
	}
	
	/* Remove a joint */
	private: void RemoveJoint(physics::JointPtr &_joint)
	{
		if (_joint)
		{
			// enable collision of the child link
			physics::LinkPtr child = _joint->GetChild();
			child->SetCollideMode("all");
			_joint->Detach();
			_joint.reset();
		}
	}
	
	/* Gives the index of the lower position error with respect to a vector of anchors */
	private: int ClosestAnchor(int _link_idx, std::vector<double>& _pos_err, 
															std::vector<double>& _rot_err, std::vector<math::Pose> _anchors)
	{
		math::Pose link_pose = this->links_[_link_idx]->GetWorldPose() - this->model_->GetWorldPose();
		for (int i = 0; i < _anchors.size(); i++)
		{
			_pos_err[i] = (_anchors[i].pos - link_pose.pos).GetLength();
			_rot_err[i] = (_anchors[i].rot.GetZAxis() - link_pose.rot.GetZAxis()).GetLength();
		}
		return std::min_element(_pos_err.begin(), _pos_err.end()) - _pos_err.begin();
	}
	
	/* Custom log messages */
  private: void BlueMsg(std::string msg) {
		gzmsg << "\033[94m" << msg << "\033[0m" << endl; }
	private: void GreenMsg(std::string msg) {
		gzmsg << "\033[92m" << msg << "\033[0m" << endl; }

};
GZ_REGISTER_MODEL_PLUGIN(DexterityTest)
}
