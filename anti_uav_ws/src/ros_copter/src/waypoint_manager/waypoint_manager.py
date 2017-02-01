#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import random
import math
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ros_copter.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from time import clock
import csv
import numpy as np

class WaypointManager():

    def __init__(self):

        # get parameters
        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 2)
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.waypoint_filename = rospy.get_param('~waypoint_filename', "/home/skyler/waypoints.csv")
        self.namespace = rospy.get_param('~namespace', 'intruder')
        self.path_type = rospy.get_param('~path_type', 'file')

        # parameters specific to intruding uav
        self.ring_distance = rospy.get_param('~ring_distance', 20)
        self.number_of_rings = rospy.get_param('~number_of_rings', 6)
        self.delta_theta_range = rospy.get_param('~delta_theta_range', math.pi / 6)

        initial_position = Vector3()
        initial_position.x = self.number_of_rings*self.ring_distance
        initial_position.y = 0
        initial_position.z = 30 # Note: this is a positive number representing magnitude of height, but in the NED coordinate system, up is negative
        self.initial_position = rospy.get_param('~initial_position', initial_position)

        reset_position = Vector3()
        reset_position.x = 130
        reset_position.y = 0
        reset_position.z = -35 # Note: this is a positive number representing magnitude of height, but in the NED coordinate system, up is negative
        self.reset_position = rospy.get_param('~reset_position', reset_position)

        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber(self.namespace + '/ground_truth/odometry', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('waypoint', Vector3, queue_size=5, latch=True)



        # Start Up Waypoint List
        self.createWaypointList()


        self.current_waypoint_index = 0

        waypoint_msg = Vector3()
        current_waypoint = np.array(self.waypoint_list[0])
        waypoint_msg.x = current_waypoint[0]
        waypoint_msg.y = current_waypoint[1]
        waypoint_msg.z = current_waypoint[2]
        self.waypoint_pub_.publish(waypoint_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            print("spinning")
            rospy.spin()

    def randomizeWaypointPathToCenter(self):
        self.waypoint_list = []

        random_seed = clock()
        random.seed(random_seed)

        previous_theta = math.atan2(self.initial_position.y, self.initial_position.x)
        for ring in range(0, self.number_of_rings):
            # Range between -90 and 90 degrees
            delta_theta = random.random() * self.delta_theta_range - self.delta_theta_range / 2
            theta = previous_theta + delta_theta
            # Calculate x and y from random theta; z steadily decreases towards the ground
            random_waypoint = []
            random_waypoint.append((self.number_of_rings - ring) * self.ring_distance * math.cos(theta))
            random_waypoint.append((self.number_of_rings - ring) * self.ring_distance * math.sin(theta))
            random_waypoint.append(-1.0*(self.initial_position.z - self.initial_position.z/self.number_of_rings*ring))

            # Store the waypoint
            self.waypoint_list.append(map(float, random_waypoint))

        final_waypoint = [0.0, 0.0, 0.0]
        self.waypoint_list.append(map(float, final_waypoint))


    def straightHighPathToCenter(self):
        # Clear out previous list
        self.waypoint_list = []
        for ring in range(0, self.number_of_rings):
            # Calculate x and y from random theta; z steadily decreases towards the ground
            waypoint = []
            waypoint.append((self.number_of_rings - ring) * self.ring_distance)
            waypoint.append(0.0)
            waypoint.append(-1.0*(self.initial_position.z - self.initial_position.z/self.number_of_rings*ring))

            # Store the waypoint
            self.waypoint_list.append(map(float, waypoint))

        final_waypoint = [0.0, 0.0, 0.0]
        self.waypoint_list.append(map(float, final_waypoint))

    def createWaypointList(self):
        self.waypoint_list = []
        self.current_waypoint_index = 0

        if self.path_type == "random":
            self.randomizeWaypointPathToCenter()

        elif self.path_type == "straight":
            self.straightHighPathToCenter()

        elif self.path_type == 'file':
            print(self.waypoint_filename)
            if self.waypoint_filename:
                file = csv.reader(open(self.waypoint_filename, 'r'))
                print("Waypoints file")
                for row in file:
                    print(map(float, row))
                    self.waypoint_list.append(map(float, row))


    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])


        reset_position = np.array([self.reset_position.x,
                                     self.reset_position.y,
                                     self.reset_position.z])
                                     
        reset_pos_error = np.linalg.norm(current_position - reset_position)

        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])

        error = np.linalg.norm(current_position - current_waypoint)

        # Restart the waypoints if we reset the position

        # If the position is back to the starting position, we have reset - also reset the waypoints.
        if reset_pos_error < 1.0:
            self.createWaypointList()

            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            waypoint_msg = Vector3()
            waypoint_msg.x = next_waypoint[0]
            waypoint_msg.y = next_waypoint[1]
            waypoint_msg.z = next_waypoint[2]
            self.waypoint_pub_.publish(waypoint_msg)
        elif error < self.threshold:
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)
            else:
                if self.current_waypoint_index > len(self.waypoint_list):
                    self.current_waypoint_index -=1
            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            waypoint_msg = Vector3()
            waypoint_msg.x = next_waypoint[0]
            waypoint_msg.y = next_waypoint[1]
            waypoint_msg.z = next_waypoint[2]
            self.waypoint_pub_.publish(waypoint_msg)


if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
