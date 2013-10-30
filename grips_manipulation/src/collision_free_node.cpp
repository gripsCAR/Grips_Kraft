// ROS
#include <ros/ros.h>

// Grips Manipulation
#include <grips_manipulation/collision_free_motion.hpp>


// Collision-free motion
grips_manipulation::CollisionFreeMotionPtr collision_free_motion_;

int main(int argc, char **argv)
{
  ros::init (argc, argv, "collision_free_node");
  ros::NodeHandle nh;
  
  // Load Collision-free motion
  collision_free_motion_.reset(new grips_manipulation::CollisionFreeMotion());
  
  // It's needed for the monitored_planning_scene
  ros::AsyncSpinner spinner(4);
  spinner.start();  // Use 4 threads
  
  ros::spin();
  
  return 0;
}
