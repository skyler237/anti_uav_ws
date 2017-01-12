#!/usr/bin/env python

### To change the publish rate, modify evartCallback.rate below

import rospy
from evart_bridge.msg import transform_plus

def main():
    # Initialize ROS
    rospy.init_node('psuedo_gps', anonymous=True)

    # Subscribers
    rospy.Subscriber("/evart/xtion/rgb", transform_plus, evartCallback)
    
    # Publishers
    global gps_pub
    gps_pub = rospy.Publisher("psuedo_gps", transform_plus)
    rospy.spin()

# filterStateCallback sets the current node id and current z value
def evartCallback(msg):
    evartCallback.rate = 1 

    current_time = rospy.get_time()
    if evartCallback.last_pub_time is None:
        evartCallback.last_pub_time = current_time 
        gps_pub.publish(msg)
        print 'publish message', current_time

    elif current_time - evartCallback.last_pub_time > evartCallback.rate:
        evartCallback.last_pub_time = current_time 
        gps_pub.publish(msg)
        print 'publish message', current_time
evartCallback.last_pub_time = None


if __name__ == '__main__':
	main()
