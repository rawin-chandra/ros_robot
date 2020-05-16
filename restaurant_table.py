#!/usr/bin/env python
# license removed for brevity

#make this code to be a ROS node
#Dev. by T. Rawin http://www.facebook.com/teacherwinzatori

import pygame
import time
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance

pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=50)

rospy.init_node('goal_check', anonymous=True)

pygame.init()  #initialize
screen = pygame.display.set_mode((1423,689))

table = pygame.image.load('/home/rawin/table.jpg')
clock = pygame.time.Clock()  #use for frame rate tune
mario_rect = pygame.Rect(112,25,22,30)
mario_rect2 = pygame.Rect(138,24,22,30)

def send_goal(num):
    global pub
    mypose = PoseStamped()
    mypose.header.seq = 0
    mypose.header.stamp = rospy.Time()
    mypose.header.frame_id = "map"
    p = Pose()
    if num == 0:
		p.position.x = 3.81240105629
		p.position.y = -0.0422300845385
		p.position.z = 0

		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = 0
		p.orientation.w = 1

		mypose.pose = p
    elif num == 1:
		p.position.x = 2.53961610794
		p.position.y = -1.09265220165
		p.position.z = 0

		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = -0.7245616579
		p.orientation.w = 0.689209985347

		mypose.pose = p
    elif num == 2:
		p.position.x = 0.890936732292
		p.position.y = 0.70980155468
		p.position.z = 0

		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = 0.667813612511
		p.orientation.w = 0.744328542342
        	mypose.pose = p

    elif num == 3:
		p.position.x = 0.742720782757
		p.position.y = -0.751610517502
		p.position.z = 0

		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = -0.732907667557
		p.orientation.w = 0.68032811998
        	mypose.pose = p
    elif num == 4:
		p.position.x = 0
		p.position.y = 0
		p.position.z = 0

		p.orientation.x = 0
		p.orientation.y = 0
		p.orientation.z = 0
		p.orientation.w = 1
		mypose.pose = p

    if num <= 4:
	       pub.publish(mypose)


while True:   #infinity loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if pos[0] > 134 and pos[0] < 632 and pos[1] > 100 and pos[1] < 287:
                print("table1")
                send_goal(0)
            elif pos[0] > 765 and pos[0] < 1258 and pos[1] > 100 and pos[1] < 287:
                print("table2")
                send_goal(1)
            elif pos[0] > 134 and pos[0] < 632 and pos[1] > 384 and pos[1] < 543:
                print("table3")
                send_goal(2)
            elif pos[0] > 770 and pos[0] < 1255 and pos[1] > 384 and pos[1] < 543:
                print("table4")
                send_goal(3)


    screen.blit(table,(0,0))

    pygame.display.flip()
    clock.tick(30)
