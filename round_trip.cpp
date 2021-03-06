#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

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

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("goal 1");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("goal 1 pass");
  else
    ROS_INFO("goal 1 failed");
//--------------------------
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.position.z = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("goal 2");
ac.sendGoal(goal);

ac.waitForResult();

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("goal 2 pass");
else
  ROS_INFO("goal 2 failed");
//-----------------------------
goal.target_pose.pose.position.x = 3.5;
goal.target_pose.pose.position.y = 0.3;
goal.target_pose.pose.position.z = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("goal 3");
ac.sendGoal(goal);

ac.waitForResult();

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("goal 3 pass");
else
  ROS_INFO("goal 3 failed");
//--------------------------------------
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.position.z = 0.0;
goal.target_pose.pose.orientation.z = 1.0;
goal.target_pose.pose.orientation.w = 0.0;

ROS_INFO("goal 4");
ac.sendGoal(goal);

ac.waitForResult();

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("goal 4 pass");
else
  ROS_INFO("goal 4 failed");
  return 0;
}
