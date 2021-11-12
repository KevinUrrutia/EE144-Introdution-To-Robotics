#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        #reset odometry to zero
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        #subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        #initial conditions
        self.previous_waypoint = np.array([0,0])
        self.previous_velocity = np.array([0,0])
        self.vel_ref = 0.3
        self.vel = Twist()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            #save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
 
    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                      [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                      [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        #generate polynomial trajectory and move to current_waypoint 
        #next_waypoint is to help determine the velocity to pass current waypoint
      
        print("*******************************************************************************")
	print(self.previous_velocity[0])
	print(self.previous_velocity[1])
        #calculate the orientation of the robot
	theta = atan2((next_waypoint[1] - self.previous_waypoint[1]),(next_waypoint[0] - self.previous_waypoint[0]))
        #theta = atan2((current_waypoint[1] - self.previous_waypoint[1]),(current_waypoint[0] - self.previous_waypoint[0]))
        
        #decompose the velocity for the boundary conditions
        #check if last waypoint
        vx_end = self.vel_ref * cos(theta)
        vy_end = self.vel_ref * sin(theta)

        #change to set the trajectory
        T = 4

        #coeffcient array calculate from polynomial function
        c_x = self.polynomial_time_scaling_3rd_order(self.previous_waypoint[0], self.previous_velocity[0], current_waypoint[0], vx_end, T)
        c_y = self.polynomial_time_scaling_3rd_order(self.previous_waypoint[1], self.previous_velocity[1], current_waypoint[1], vy_end, T)
        calc_vx = 0
        calc_vy = 0
        #calculate the changing velocity based on time
        for i in range(T * 10 + 1):
            calc_vx = c_x[2] + 2*c_x[1]*(i/10) + 3*c_x[0]*pow((i/10),2)
            calc_vy = c_y[2] + 2*c_y[1]*(i/10) + 3*c_y[0]*pow((i/10),2)
	    velocity  = sqrt(pow(calc_vy, 2) + pow(calc_vx, 2))
            self.vel.linear.x = velocity

            vel_theta = atan2(calc_vy, calc_vx)

            #compute the delta theta
            diff_theta = vel_theta - self.pose.theta
            #change the values of Kp
            kp = 10    
            P_term = kp * diff_theta
            #print(P_term)

            self.vel.angular.z = P_term
            #self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

	print(calc_vx[0])
	print(calc_vy[0])
        self.previous_waypoint = current_waypoint
	print(current_waypoint[0])
	print(self.previous_waypoint[0])

        self.previous_velocity = [calc_vx, calc_vy]
	print(self.previous_velocity[0])
	print(self.previous_velocity[1])

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        #input: p,v: postion and velocity of start/end point
        #       T: the desired time to complete this segment of trajectory (seconds)
        #output: the coefficients of this polynomial
        x_array = np.array([[p_start], [p_end], [v_start], [v_end]]) #initial condition array
        #print(x_array)
        T_mat = np.array([[0,0,0,1], [pow(T,3),pow(T,2),T,1], [0,0,1,0], [3*pow(T,2),2*T,1,0]]) # coefficient matrix
        #print(T_mat)
 
        T_mat_inv = np.linalg.inv(T_mat) #find the inverse coefficient matrix
        coefficients = np.dot(T_mat_inv, x_array)  #solve for the coefficient array 
        
        return coefficients #return coefficient array
   
    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))     



if __name__ == '__main__':
    whatever = Turtlebot()
