#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np
import time

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point #reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        #calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Crtl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        #reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        #subscribe to Odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            #save trajectory in .csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        angle = pi/2
	vel = Twist()
        PID = Controller()
        PID.setPoint(angle)
        PID.setPD(0.99, 0.70)

        i = 0
        while not rospy.is_shutdown():
            if(i < 4):
                for j in range(74):
                    vel.linear.x = 0.5
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    self.rate.sleep()

		time.sleep(3)

                while((abs(PID.previous_error) > 0.01) or (PID.previous_error == 0.0)):
                    print('-----------------------------')
                    PID_update = PID.update(self.pose.theta)
                    vel.linear.x = 0
                    vel.angular.z = PID_update
                    self.vel_pub.publish(vel)
                    self.rate.sleep()

                i +=1
                rospy.logwarn(i)
		if (i == 2):
		    angle = -pi/2
                    print("++++++++++++++++++++++++++")
		else:
                    angle += pi/2

		PID.setPoint(angle)
            else:
                break

    def odom_callback(self, msg):
        #get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        #logging once every 100 times (Gazebo runs at 100Hz; we  save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) #save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) + ";" + " y=" + str(self.pose.y) + ";" + " theta=" + str(yaw))

if __name__ == '__main__':
    whatever = Turtlebot()
