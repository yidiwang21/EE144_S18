#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May 8, 20:47 2018
Description: This script is for demo at point 2. The robot must be facing x axis initially

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
''' waypoint of a half circle '''
# waypoints = np.array([[0,0], [0.012,0.156], [0.049,0.309], [0.109,0.454], [0.191,0.588], [0.293,0.707], [0.412,0.809], [0.546,0.891], [0.691,0.951], [0.844, 0.988], [1,1]])
waypoints = np.array([[0,0], [0,-0.5], [0,-1]])
curr_point = np.array([0])
next_point = np.array([0])
ptr = 0
points_num = len(waypoints)
curr_point = waypoints[ptr]
next_point = waypoints[ptr]
face_orientation = 0.0

EPSILON = 0.2   # FIXME
dist_thresh = 0.15   # FIXME
RAD = 2 * pi / 360
kp = 1              # FIXME: set an estimated proper kp value
v_ang = 80          # FIXME
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
        print ("Start turning to the next point...")
        global ptr
        global next_point
        global face_orientation

        current_angle = 0
        angular_speed = v_ang * RAD             # NOTE: the value will be different on different material
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        rate = rospy.Rate(100)              # NOTE: set to be 100 in real test

        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        curr_orientation = orientation[2]
        print('curr_orientation = ', orientation[2])
        if abs(curr_point[1] - next_point[1]) <= EPSILON and abs(curr_point[0] - next_point[0]) <= EPSILON: # point unchanged
            target_angle = 0
        elif abs(curr_point[1] - next_point[1]) <= EPSILON:   # position at y unchanged, moving on x-axis
            if curr_point[0] <= next_point[0]:
                target_angle = - curr_orientation
            else:
                target_angle = pi - curr_orientation
        elif abs(curr_point[0] - next_point[0]) <= EPSILON:   # position at x unchanged, moving on y-axis
            if curr_point[1] <= next_point[1]:
                target_angle = pi / 2 - curr_orientation
            else:
                target_angle = 3 * pi / 2 - curr_orientation
        else:
            target_angle = atan((next_point[0] - position[0]) / (next_point[1] - position[1])) - curr_orientation
            # print('init target_angle = ', target_angle)
            if curr_point[0] > next_point[0]:
                target_angle = pi + target_angle
        print('target_angle = ', target_angle)
        relative_angle = target_angle - curr_orientation + face_orientation
        # print('init relative_angle = ', relative_angle)
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
        print('relative_angle = ', relative_angle)
        # if angular_speed < 0:
        #     print('truning clockwise')
        # else:
        #     print('truning counterclockwise')

        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_sec()
            cnt = 0
            while abs(current_angle - relative_angle) > EPSILON:
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t1 - t0)
            break

        self.updateFacingAng()
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
        rate = rospy.Rate(100)      # NOTE: set to be 100 in real test

        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        cnt = 0
        while sqrt((next_point[1] - position[1])**2 + (next_point[0] - position[0])**2) > dist_thresh:
        # Find the position of the current goal waypoint in the robot’s coordinate frame, PR.
        # Find the angle θ that PR forms with the current direction of motion of the robot
            (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
            orientation = tf.transformations.euler_from_quaternion(quaternion)
            # there is no information collection for the sign of theta, so using atan2 here
            theta = atan2(next_point[0] - position[0], next_point[1] - position[1])
            vel.angular.z = kp * theta * 2 * pi/360
            # vel.linear.x = -abs(theta) * 0.002 + 0.1    # FIXME: untested in reality
            # vel.linear.x = 0.12
            vel.linear.x = 0.2                            # FIXME
            self.set_velocity.publish(vel)
            rate.sleep()
            if cnt % 3 == 0:
                x = np.append(x, position[0])
                y = np.append(y, position[1])
            cnt = cnt + 1

        self.updateFacingAng()
        rospy.sleep(1)

    def updateFacingAng(self):
        global face_orientation
        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        face_orientation = orientation[2]
        print ('face_orientation = ', face_orientation)

    def shutdown(self):
        rospy.loginfo("Stop Action")
        stop_vel = Twist()
        stop_vel.linear.x = 0
        stop_vel.angular.z = 0
        self.set_velocity.publish(stop_vel)
        rospy.sleep(1)

    def main(self):
        self.turnToPoint()
        self.moveToPoint()

if __name__ == '__main__':
    try:
        ins = turtlebot_move()
        while ptr < points_num - 1:
            curr_point = waypoints[ptr]
            next_point = waypoints[ptr + 1]
            print ('==============================================')
            print('curr_point', + curr_point)
            print('next_point', + next_point)
            ins.main()
            ptr = ptr + 1
            print ('==============================================')
        # plt.scatter(x, y)
        # plt.title('Trajectory of the Given Test Case')
        # plt.show()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Action terminated.")
