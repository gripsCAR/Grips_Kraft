/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, UPM
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
 *   * Neither the name of the UPM nor the names of its
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

/* Author: Francisco Suarez-Ruiz
   Desc:   Collision-free motion for the grips manipulator
*/

#ifndef GRIPS_MANIPULATION__COLLISION_FREE_MOTION_
#define GRIPS_MANIPULATION__COLLISION_FREE_MOTION_

// ROS
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Messages
#include <geometry_msgs/Pose.h>
#include <grips_msgs/GripsState.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace grips_manipulation
{
class CollisionFreeMotion
{
private:
  // A shared node handle
  ros::NodeHandle nh_, nh_private_;
  
  // Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit_msgs::PlanningScene scene_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_;
  
  // Parameters
  std::string robot_description_;
  
public:
  // Constructor
  CollisionFreeMotion();
  // Destructor
  ~CollisionFreeMotion();
  
}; // end class

  typedef boost::shared_ptr<CollisionFreeMotion> CollisionFreeMotionPtr;
  typedef boost::shared_ptr<const CollisionFreeMotion> CollisionFreeMotionConstPtr;

} // end namespace

#endif
