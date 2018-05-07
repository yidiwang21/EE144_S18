#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty
from std_msgs.msg import String, Empty
from time import time
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt

PI = 3.1415926535897
RAD = 2 * PI / 360
PHI = 0
kp = 15     # FIXME: this need to be adjust
kd = 1000   # FIXME: this need to be adjust

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

        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
        # reset odometry
        reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
        # this message take a few iterations to get through
        timer = time()
        while time() - timer < 1.0:
            reset_odom.publish(Empty())

        rospy.loginfo("Finished Initializing!")

    def moveForward(self):
        rospy.loginfo("Start moving forward...")
        global x, y

        vel = Twist()
        vel.linear.x = 0.5
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        # TODO: use the position change to do the feedback
        (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        # FIXME: try to use a const phi value
        init_orientation = orientation[2]
        temp_orientation = init_orientation
        print 'init_orientation = ', init_orientation

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t1 = rospy.Time.now().to_sec()
            t = rospy.Time(t1 + 10)
            print("Current time:", t1)
            cnt = 0
            while t1 < t.to_sec():
                # get current position and orientation
                (position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
                orientation = tf.transformations.euler_from_quaternion(quaternion)
                # rospy.loginfo("position: " + str(position))
                # rospy.loginfo("orientation: " + str(orientation))
                if cnt % 3 == 0:    # 30 dots per side
                    x = np.append(x, position[0])
                    y = np.append(y, position[1])
                cnt = cnt + 1
                # get the orientation difference and derivative difference of z-axis
                det_phi = init_orientation - orientation[2]
                d_phi = temp_orientation - orientation[2]
                temp_orientation = orientation[2]
                # p and d controller for reset angular speed of z-axis each loop
                vel.angular.z = (kp * det_phi + kd * d_phi) * RAD
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
            break                               # after 10 secs, end moving forward

        rospy.sleep(1)

    def turnRight(self):
        rospy.loginfo("Start turning right...")

        angular_speed = 10 * RAD             # pick a proper angular speed
        relative_angle = 90 * RAD            # set target turning angle to 90 degrees

        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angular_speed

        rate = rospy.Rate(10)
        current_angle = 0
        print("current_angle = ", current_angle / RAD)
        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_sec()
            cnt = 0
            while current_angle < relative_angle:
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t1 - t0)
            break

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
        i = 0
        while i < 4:            # iter 4 times to complete a square
            ins.moveForward()
            ins.turnRight()
            i = i + 1
            PHI = i * PI / 2
        rospy.loginfo("Finished Moving 5x5")
        plt.scatter(x, y)
        plt.show()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Action terminated.")
