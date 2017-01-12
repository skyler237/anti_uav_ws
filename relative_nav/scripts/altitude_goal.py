#!/usr/bin/env python

import rospy
import numpy
import time
from relative_nav.msg import Goal
from relative_nav.msg import FilterState
import argparse
import math

node_id = -1
node_id_set = False

current_x = 0
current_y = 0
current_yaw = 0

parser = argparse.ArgumentParser(description='Command an altitude')
parser.add_argument('Z',type=float, help='z')
args = parser.parse_args()

node_name = 'goal_python_publisher'
current_topic = "current_state"
goal_topic = "goal"

def QuaterniontoYaw(w,x,y,z):
	yaw = math.atan2(2*(w*z + x*y),(w*w + x*x - y*y - z*z)) 
	return yaw

def filterStateCallback(msg):
	global node_id
	global node_id_set
	global current_x
	global current_y
	node_id = msg.node_id
	node_id_set = True
	current_x = msg.transform.translation.x
	current_y = -msg.transform.translation.y

	w = msg.transform.rotation.w
	x = msg.transform.rotation.x
	y = -msg.transform.rotation.y
	z = -msg.transform.rotation.z

	current_yaw = QuaterniontoYaw(w,x,y,z)

rospy.init_node(node_name, anonymous=True)
rospy.Subscriber(current_topic, FilterState,  filterStateCallback)
goal_pub = rospy.Publisher(goal_topic, Goal)

while not node_id_set:
	print "if you see this a lot, then it is the source of our delayed altitude command"
	time.sleep(0.1)

goal = Goal()
goal.node_id = node_id
goal.pose.x = current_x 
goal.pose.y = current_y
goal.pose.z = args.Z 
goal.pose.yaw = current_yaw 

rate = rospy.Rate(1.0)
rate.sleep() #Needs to wait a little bit??
goal_pub.publish(goal)

print 'done'

