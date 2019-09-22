#!/usr/bin/env python
# license removed for brevity
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math
import time



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

     
     #vel_msg = Twist()

     #pub.publish(vel_msg)   


if __name__ == "__main__":
    global model
    rospy.init_node('duck_bot', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)     
    
    time.sleep(0.03)
    rospy.spin()
    












    
