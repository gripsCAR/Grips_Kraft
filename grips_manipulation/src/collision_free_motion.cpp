/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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


#include <grips_manipulation/collision_free_motion.hpp>

namespace grips_manipulation
{
// Constructor
CollisionFreeMotion::CollisionFreeMotion():
  nh_private_("~")
{
  // get the robot_description parameter from the closest namespace
  if (!nh_private_.searchParam("robot_description", robot_description_))
  {
    ROS_FATAL_NAMED("collision","Failed to find the [robot_description] in the parameter server");
    return;
  }
  // Create planning scene monitor
  ROS_INFO_NAMED("collision", "Using [%s] parameter as robot_description", robot_description_.c_str());
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_));
  planning_scene_ = planning_scene_monitor_->getPlanningScene();
  if (planning_scene_)
  {
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->setPlanningScenePublishingFrequency(50.0); // Hz
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  }
  else
    ROS_FATAL_NAMED("collision","Planning scene not configured");
  
  // CurrentStateMonitor instance
  current_state_ = planning_scene_monitor_->getStateMonitor();
  
  /* Check collisions
  if (planning_scene_->isStateColliding())
    ROS_INFO_NAMED("collision","Collision detected"); */
}

// Destructor
CollisionFreeMotion::~CollisionFreeMotion()
{
}

} // end namespace
