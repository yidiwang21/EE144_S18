#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May 8, 20:47 2018

@author: Yidi Wang
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from time import time
from math import cos, sin, tan, acos, asin, atan, sqrt
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt

#these waypoints are given as list for convience, however, you can use any data type that you like
#These coordinates are in the "world" coordinate frame
waypoints = np.array([[0,0],[0.5,0],[1,0],[1,0],[1,0.5],[1,1],[1,1],[0.5,1],[0,1],[0,1],[0,0.5],[0,0]])
curr_point = np.array([0])
next_point = np.array([0])
ptr = 0
points_num = 12

EPSILON = 1e-3  # error tol
dist_thresh = 0.01  # FIXME
PI = 3.1415926535897
kp = 15     # FIXME: set an estimated proper kp value
x = np.array([0])
y = np.array([0])

# Initialize the tf listener
tfListener = tf.TransformListener()

class turtlebot_move():
    def __init__(self):
        # reset the position of the robot to the center of the world for easier observaton
        rospy.init_node('reset_world', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.shutdown)
        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10) # reset odometry
        timer = time()
        while time() - timer < 1.0:
            reset_odom.publish(Empty())
        rospy.loginfo("Finished Initializing!")

    def move2NextPoint(self):
        rospy.loginfo("Start moving to the next point...")
        global x, y
        global ptr
        # initialize velocities to 0
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        curr_point = np.apped(waypoints[ptr])
        next_point = np.apped(waypoints[ptr + 1])

        while sqrt((next_point[1] - position[1])**2 + (next_point[0] - position[0])**2) > dist_thresh:
            # Find the position of the current goal waypoint in the robot’s coordinate frame, PR.
            # Find the angle θ that PR forms with the current direction of motion of the robot
            (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
            orientation = tf.transformations.euler_from_quaternion(quaternion)
            theta = atan2(next_point[1] - position[1], next_point[0] - position[0])
            vel.angular.z = kp * theta * 2 * PI/360
            vel.linear.x = 5    # FIXME
            # save points pairs to plot
            if cnt % 3 == 0:
                x = np.append(x, position[0])
                y = np.append(y, position[1])
            cnt = cnt + 1

        ptr = ptr + 1

    def shutdown(self):
        rospy.loginfo("Stop Action")
        stop_vel = Twist()
        stop_vel.linear.x = 0
        stop_vel.angular.z = 0
        self.set_velocity.publish(stop_vel)

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ins = turtlebot_move()
        while ptr < points_num:
            move2NextPoint()
        rospy.loginfo("Finished reset path")
        plt.scatter(x, y)
        plt.show()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Action terminated.")
