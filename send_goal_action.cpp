/*
code for set the goal of robot in a map (room)

*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){ 

  ros::init(argc, argv, "my_navigation_goals");
  ros::NodeHandle n;
  
  ros::Duration(20).sleep();
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 4.54307174683;
  goal.target_pose.pose.position.y = -0.244015842676;
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0.685251765044;
  goal.target_pose.pose.orientation.w = 0.728306266967;

  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal 1");
  else
    ROS_INFO("The base failed to move to goal 1");

  
  ros::Duration(5).sleep();

  //-------------------------------------------------------------
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 5;	 		
  goal.target_pose.pose.position.y = 0.5;  
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending goal 2");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal 2");
  else
    ROS_INFO("The base failed to move to goal 2");



  return 0;
}
