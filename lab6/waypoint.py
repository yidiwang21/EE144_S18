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
from math import cos, sin, tan, acos, asin, atan, atan2, sqrt, pi
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt

#these waypoints are given as list for convience, however, you can use any data type that you like
#These coordinates are in the "world" coordinate frame
# waypoints = np.array([[0,0],[0.5,0],[1,0],[1,0],[1,0.5],[1,1],[1,1],[0.5,1],[0,1],[0,1],[0,0.5],[0,0]])
# a set of arbitary points for testing
waypoints = np.array([[0,0],[0.5,-0.5]])
curr_point = np.array([0])
next_point = np.array([0])
ptr = 0
points_num = len(waypoints)
curr_point = waypoints[ptr]
next_point = waypoints[ptr]
face_orientation = 0.0

EPSILON = 0.05
dist_thresh = 0.1  # FIXME
RAD = 2 * pi / 360
kp = 1     # FIXME: set an estimated proper kp value
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

    def turnToPoint(self):
        print ("Start turning...")
        global ptr
        global next_point
        global face_orientation

        current_angle = 0
        angular_speed = 10 * RAD             # pick a proper angular speed
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        rate = rospy.Rate(10)

        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        curr_orientation = orientation[2]
        print('curr_orientation', + orientation[2])
        face_angle = atan2(curr_point[0] - next_point[0], curr_point[1] - next_point[1])
        if abs(position[1] - next_point[1]) <= EPSILON and abs(position[0] - next_point[0]) <= EPSILON: # point unchanged
            target_angle = curr_orientation
        elif abs(position[1] - next_point[1]) <= EPSILON:   # position at y unchanged, moving on x-axis
            if position[0] <= next_point[0]:
                target_angle = - curr_orientation
            else:
                target_angle = pi - curr_orientation
        elif abs(position[0] - next_point[0]) <= EPSILON:   # position at x unchanged, moving on y-axis
            if position[1] <= next_point[1]:
                target_angle = pi / 2 - curr_orientation
            else:
                target_angle = 3 * pi / 2 - curr_orientation
        else:
            # FIXME
            print ('shit')
            target_angle = atan2(next_point[0] - position[0], next_point[1] - position[1])
            if position[0] > next_point[0]:
                target_angle = pi + target_angle
        print('target_angle', + target_angle)
        relative_angle = target_angle - curr_orientation + face_orientation
        print('init relative_angle', + relative_angle)
        if relative_angle < 0:
            relative_angle = - relative_angle
            vel.angular.z = - angular_speed
        elif relative_angle > pi and relative_angle < 2 * pi:
            relative_angle = 2 * pi - relative_angle
            vel.angular.z = - angular_speed
        elif relative_angle >= 2 * pi:
            relative_angle = relative_angle - 2 * pi
            vel.angular.z = angular_speed
        else:
            vel.angular.z = angular_speed
        print('relative_angle', + relative_angle)

        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_sec()
            cnt = 0
            while abs(current_angle - relative_angle) > EPSILON:
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t1 - t0)
                # if cnt % 30 == 0:
                    # print('current_angle', + current_angle)
                # cnt = cnt + 1
            break

        rospy.sleep(1)

    def moveToPoint(self):
        print ("Start moving to the next point...")
        global x, y
        global ptr
        global next_point
        global face_orientation
        # initialize velocities to 0
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        rate = rospy.Rate(10)

        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        cnt = 0
        while sqrt((next_point[1] - position[1])**2 + (next_point[0] - position[0])**2) > dist_thresh:
        # Find the position of the current goal waypoint in the robot’s coordinate frame, PR.
        # Find the angle θ that PR forms with the current direction of motion of the robot
            (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
            orientation = tf.transformations.euler_from_quaternion(quaternion)
            theta = atan2(next_point[1] - position[1], next_point[0] - position[0])
            vel.angular.z = kp * theta * 2 * pi/360
            vel.linear.x = 0.1    # FIXME
            self.set_velocity.publish(vel)
            rate.sleep()
            # save points pairs to plot
            if cnt % 3 == 0:
                x = np.append(x, position[0])
                y = np.append(y, position[1])
            cnt = cnt + 1

        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        face_orientation = orientation[2]
        # face_orientation = int(orientation[2] / (pi / 2)) * pi / 2
        print ('face_orientation = ', face_orientation)
        print ('==============================================')
        rospy.sleep(1)

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
        while ptr < points_num - 1:
            curr_point = waypoints[ptr]
            next_point = waypoints[ptr + 1]
            ptr = ptr + 1
            print ('==============================================')
            print('curr_point', + curr_point)
            print('next_point', + next_point)
            ins.turnToPoint()
            ins.moveToPoint()

        plt.scatter(x, y)
        plt.show()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Action terminated.")
