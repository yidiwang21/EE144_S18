#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

PI = 3.1415926535897

class turtlebot_move():
    def __init__(self):
        rospy.init_node('reset_world', anonymous=False)
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        if reset_world():
            rospy.loginfo("World successfully reset!")
        else:
            rospy.loginfo("World failed to reset!")

        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.shutdown)

        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.loginfo("Initializing... Please reset sim time in the GUI...")
        rospy.sleep(5)
        rospy.loginfo("Finished Initializing!")

    def moveForward(self):
        rospy.loginfo("Start moving forward...")

        vel = Twist()
        vel.linear.x = 0.5
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t1 = rospy.Time.now().to_sec()
            t = rospy.Time(t1 + 10)
            flag = 0
            print("Current time:", t1)
            while t1 < t.to_sec():
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                if flag % 10 == 0:
                    print("Current time:", t1)
                flag = flag + 1
            break

        rospy.sleep(1)

    def turnRight(self):
        rospy.loginfo("Start turning right...")

        angular_speed = 10*2*PI/360
        relative_angle = 90*2*PI/360

        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angular_speed

        rate = rospy.Rate(10)
        current_angle = 0
        print("current_angle = ", current_angle*180/PI)
        while not rospy.is_shutdown():
            t0 = rospy.Time.now().to_sec()
            flag = 0
            while current_angle < relative_angle:
                self.set_velocity.publish(vel)
                rate.sleep()
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t1-t0)
                if flag % 10 == 0:
                    print("Current angle = ", current_angle*180/PI)
                flag = flag + 1
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
        ins.moveForward()
        ins.turnRight()
        ins.moveForward()
        rospy.loginfo("Finished Moving 5x5")
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Action terminated.")
