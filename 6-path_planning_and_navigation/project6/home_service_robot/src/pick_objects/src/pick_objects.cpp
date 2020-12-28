#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -0.4;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO_STREAM("Sending goal to the pickup zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("The robot reached the pickup zone");
  else
  {
    ROS_ERROR_STREAM("The robot failed to reach the pickup zone");
    return 1;
  }

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.position.y = -0.6;
  // Oriented to y-
  goal.target_pose.pose.orientation.z = -0.7068252;
  goal.target_pose.pose.orientation.w = 0.7073883;

  ROS_INFO_STREAM("Sending goal to the drop off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("The robot reached the drop off zone");
  else
  {
    ROS_ERROR_STREAM("The robot failed to reach the drop off zone");
    return 1;
  }

  return 0;
}
