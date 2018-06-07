/*
  code for set goal of robot to move around the house , 10 rounds

*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){ 

  ros::init(argc, argv, "my_navigation_goals");
  ros::NodeHandle n;
  
  ros::Duration(30).sleep();
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
 
  move_base_msgs::MoveBaseGoal goal;

for(int i=0;i<10;i++) {
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";	//sofa
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
  goal.target_pose.header.frame_id = "map";    //sofa2
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.57629609108;	 		
  goal.target_pose.pose.position.y = 0.133019387722;  
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0.700616264152;
  goal.target_pose.pose.orientation.w = 0.713538261346;

  ROS_INFO("Sending goal 2");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal 2");
  else
    ROS_INFO("The base failed to move to goal 2");

ros::Duration(5).sleep();

  //-------------------------------------------------------------
  goal.target_pose.header.frame_id = "map";    //refrigerator
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.0277309417725;	 		
  goal.target_pose.pose.position.y = 1.53114390373;  
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0.999997804551;
  goal.target_pose.pose.orientation.w = -0.00209544566457;

  ROS_INFO("Sending goal 3");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal 3");
  else
    ROS_INFO("The base failed to move to goal 3");

  ros::Duration(5).sleep();

  
}

  return 0;
}
