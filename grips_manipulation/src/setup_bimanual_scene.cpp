// ROS
#include <ros/ros.h>

// Grips Manipulation
#include <grips_manipulation/collision_free_motion.hpp>

// ROS publishers
ros::Publisher collision_obj_pub_;

void addTable()
{
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
  // Table front_r_leg
  pose.position.x = 0.735 + 0.5;
  pose.position.y = 0.265 - 0.15;
  pose.position.z = 0.3995;
  primitive.dimensions[0] = 0.07;
  primitive.dimensions[1] = 0.07;
  primitive.dimensions[2] = 0.781;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  // Table front_l_leg
  pose.position.x = -0.735 + 0.5;
  pose.position.y = 0.265 - 0.15;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  // Table back_r_leg
  pose.position.x = 0.735 + 0.5;
  pose.position.y = -0.265 - 0.15;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  // Table back_l_leg
  pose.position.x = -0.735 + 0.5;
  pose.position.y = -0.265 - 0.15;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);
  // Add the table to the planning scene
  table.operation = table.ADD;
  ROS_INFO_NAMED("collision","Publishing table into the scene");
  collision_obj_pub_.publish(table);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "setup_bimanual_scene");
  ros::NodeHandle nh;
  // CollisionObject topic used to publish Objects into the scene
  collision_obj_pub_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_obj_pub_.getNumSubscribers() < 1 && ros::ok())
    ros::Duration(0.1).sleep();
  // Add table into the planning scene
  addTable();
  return 0;
}
