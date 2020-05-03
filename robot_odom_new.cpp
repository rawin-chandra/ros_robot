#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
 #include "std_msgs/String.h"

//float v_linear,v_angular;
float SVL,SVR;

void callback(const geometry_msgs::Twist::ConstPtr& odom_msg)
  {
	 SVL = odom_msg->linear.x;
   	 SVR = odom_msg->linear.y;  
   }


int main(int argc, char** argv){
 ros::init(argc, argv, "odometry_publisher");

ros::NodeHandle n;
 
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber sub_left = n.subscribe("ard_odom", 10, callback);

  double DL,DR;
  double dxy,dth;
  double delta_x, delta_y, delta_th;
  double v_linear,v_angular;
  double dt;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

     double vx = 0.1;
     double vy = -0.1;
     double vth = 0.1;

     ros::Time current_time, last_time;
     current_time = ros::Time::now();
     last_time = ros::Time::now();

     ros::Rate r(20.0);
     while(n.ok()){

       ros::spinOnce();               // check for incoming messages

	current_time = ros::Time::now();
	dt = (current_time - last_time).toSec();

		DL = dt * SVL;
		DR = dt * SVR;

		dxy = (DL + DR) / 2;
		dth = (DR - DL) / 0.45;         //width of robot, between 2 wheels, in meters

		x += dxy * cos(th);
	        y += dxy * sin(th);
	        th += dth;


       //since all odometry is 6DOF we'll need a quaternion created from yaw
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

       //first, we'll publish the transform over tf
       geometry_msgs::TransformStamped odom_trans;
       odom_trans.header.stamp = current_time;
       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_link";

       odom_trans.transform.translation.x = x;
       odom_trans.transform.translation.y = y;
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = odom_quat;

       //send the transform
       odom_broadcaster.sendTransform(odom_trans);

       //next, we'll publish the odometry message over ROS
       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";

       //set the position

       odom.pose.pose.position.x = x;
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = odom_quat;

       //set the velocity
       odom.child_frame_id = "base_link";
       odom.twist.twist.linear.x = dxy / dt;	 
       odom.twist.twist.linear.y = 0;	 
       odom.twist.twist.angular.z = dth / dt;     

       //publish the message
       odom_pub.publish(odom);

       last_time = current_time;

       r.sleep();
     }
   }
