#!/usr/bin/env python

from __future__ import division
import rospy
from numpy import linspace
import numpy
import matplotlib.pyplot as plt
import time
from relative_nav.msg import DesiredState
from relative_nav.msg import Goal
from relative_nav.msg import FilterState
from std_msgs.msg import Float64
from random import randint
from collections import deque
import argparse
from random import random

node_id = 1
height = 0;

def transform(data):
	return data

def heightCallback(msg):
	global height
	height = transform(msg.data)

rospy.init_node("reference_step_server", anonymous=True)

reference_step_plot_pub = rospy.Publisher("reference_step_plot", Float64, latch=True)
rospy.Subscriber("reference_step", Float64,  heightCallback)
rate_long  = rospy.Rate(1.0)
rate_short = rospy.Rate(20)
rate_long.sleep() #Needs to wait a little bit??

reference_step_plot = Float64()
reference_step_plot.data = height
reference_step_plot_pub.publish(reference_step_plot)

while not rospy.is_shutdown():
	reference_step_plot_pub.publish(reference_step_plot)
	reference_step_plot.data = height
	reference_step_plot_pub.publish(reference_step_plot)
	rate_short.sleep()
print 'done'
