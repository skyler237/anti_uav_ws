#!/usr/bin/env python

from __future__ import division
import rospy
import time
from std_msgs.msg import Float64

node_name = 'reference_stepper'
topic = "reference_step"

rospy.init_node(node_name, anonymous=True)

publisher = rospy.Publisher(topic, Float64)
rate = rospy.Rate(1.0)
rate.sleep() #Needs to wait a little bit??

height_msg = Float64()
while not rospy.is_shutdown():
	z_in = raw_input("Reference Step Height: ")
	try:
		z = float(z_in)
                z = -z/100.0
		height_msg.data = z
		publisher.publish(height_msg)
	except ValueError:
		pass
print 'done'
