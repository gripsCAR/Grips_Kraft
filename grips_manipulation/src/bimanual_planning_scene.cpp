// ROS
#include <ros/ros.h>

// Grips Manipulation
#include <grips_manipulation/collision_free_motion.hpp>

// ROS publishers
ros::Publisher left_obj_pub_, right_obj_pub_;

void addTable()
{
  // CollisionObject topic used to publish Objects into the scene
  while(collision_obj_pub_.getNumSubscribers() < 1 && ros::ok())
    ros::Duration(0.1).sleep();
  // Create the table where the robots are fixed
  moveit_msgs::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  // Table surface
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.15;
  pose.position.z = 0.795;
  pose.orientation.w = 1.0;
  primitive.type = primitive.BOX;  
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.6;
  primitive.dimensions[1] = 0.6;
  primitive.dimensions[2] = 0.01;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  // TODO: Add table legs
  // Add the table to the planning scene
  table.operation = table.ADD;
  ROS_INFO_NAMED("collision","Publishing table into the scene");
  collision_obj_pub_.publish(table);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "bimanual_collision_free");
  ros::NodeHandle nh;
  left_obj_pub_ = nh.advertise<moveit_msgs::CollisionObject>("/left_arm/collision_object", 1);
  right_obj_pub_ = nh.advertise<moveit_msgs::CollisionObject>("/right_arm/collision_object", 1);
  
  // Load Collision-free motion
  collision_free_motion_.reset(new grips_manipulation::CollisionFreeMotion());
  
  // It's needed for the monitored_planning_scene
  ros::AsyncSpinner spinner(4);
  spinner.start();  // Use 4 threads
  
  ros::spin();
  return 0;
}
