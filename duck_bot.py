#!/usr/bin/env python
# license removed for brevity
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math
import time
from tf.transformations import euler_from_quaternion


pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        
    
count = 0

def callback(data):        
     global count
     count = count + 1
     

     min = data.ranges[0]
     if min == 0:
         min = 3.0
     
     deg_min = 5
     for i in range(0,360,5):
         temp = data.ranges[i]         
         if temp != 0:
             if(temp < min):
                 min = temp
                 deg_min = i
                 print ("MIN = " , min , " at " , i)

     
     vel_msg = Twist()

     if deg_min == 5:
         if min > 0.2:
            vel_msg.linear.x = 0.2
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
         else:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

     else:
         if deg_min > 5 and deg_min < 180:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1.0
         elif deg_min >= 180:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = -1.0

     pub.publish(vel_msg)
   


if __name__ == "__main__":
    global model
    rospy.init_node('duck_bot', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)     
    
    time.sleep(0.03)
    rospy.spin()
    












    
