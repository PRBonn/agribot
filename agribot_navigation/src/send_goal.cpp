#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <sstream>

// msgs:MoveBaseAction
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_goals_node");
  // Initialize Move base client
  MoveBaseClient ac("move_base", true);
  // waiting for server to start
  while (!ac.waitForServer(ros::Duration(60))) {
    ROS_INFO("waiting for the move_base action server");
  }
  // Declaring move base goal
  move_base_msgs::MoveBaseGoal goal;

  // Setting targer frame id and time in the goal action
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Retrieving pose for command line other vice execute a defualt value
  try {
    goal.target_pose.pose.position.x = atof(argv[1]);
    goal.target_pose.pose.position.y = atof(argv[2]);

    // Convert the Euler angle to quaternion
    double radians = atof(argv[3]) * (M_PI / 180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    goal.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to: x = %f, y = %f, theta = %f", atof(argv[1]),
             atof(argv[2]), atof(argv[3]));
  } catch (const std::exception& e) {
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;
  }
  ROS_INFO("Sending move base a goal...");

  // Sending goal
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot has arrived to the goal postion");
  } else {
    ROS_INFO("The base failed for some reason...");
  }
  return 0;
}