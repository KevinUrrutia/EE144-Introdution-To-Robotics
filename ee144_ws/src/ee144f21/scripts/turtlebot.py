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
        self.vel_ref = 0.2
        self.vel = Twist()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            #save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        start = (0, 0) 
        goal = (3, 3)
        obstacles = [(2, 1), (2, 2), (3, 1), (0, 3)]
        waypoints = self.get_path_from_A_star(start, goal, obstacles)
        waypoints.insert(len(waypoints), waypoints[len(waypoints) - 1])
        print(waypoints)

        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1], i, (len(waypoints)-1))
            print("Current waypoint")
            print(waypoints[i])

    def move_to_point(self, current_waypoint, next_waypoint, count, length):
        #generate polynomial trajectory and move to current_waypoint 
        #next_waypoint is to help determine the velocity to pass current waypoint

	print("previous_velocity_x_init: ") 
        print(self.previous_velocity[0])
	print("previous_velocity_y_init: ") 
        print(self.previous_velocity[1]) 
        print(count)   

        #calculate the orientation of the robot
	theta = atan2((next_waypoint[1] - self.previous_waypoint[1]),(next_waypoint[0] - self.previous_waypoint[0]))
        #theta = atan2((current_waypoint[1] - self.previous_waypoint[1]),(current_waypoint[0] - self.previous_waypoint[0]))

        #decompose the velocity for the boundary conditions
        #check if last waypoint
        vx_end = self.vel_ref * cos(theta)
        vy_end = self.vel_ref * sin(theta)
        if(count == length - 1):
            vx_end = 0
            vy_end = 0

        #change to set the trajectory
        T = 4
        #coeffcient array calculate from polynomial function
        c_x = self.polynomial_time_scaling_3rd_order(self.previous_waypoint[0], self.previous_velocity[0], current_waypoint[0], vx_end, T)
        c_y = self.polynomial_time_scaling_3rd_order(self.previous_waypoint[1], self.previous_velocity[1], current_waypoint[1], vy_end, T)
        calc_vx = 0
        calc_vy = 0

        #calculate the changing velocity based on time
        for i in range(T * 10):
            calc_vx = c_x[2] + 2*c_x[1]*(i/10) + 3*c_x[0]*pow((i/10),2)
            calc_vy = c_y[2] + 2*c_y[1]*(i/10) + 3*c_y[0]*pow((i/10),2)
	    velocity  = sqrt(pow(calc_vy, 2) + pow(calc_vx, 2))
            self.vel.linear.x = velocity

            #calculate the angle between the velocities as a function of time
            vel_theta = atan2(calc_vy, calc_vx)

            #compute the delta theta
            diff_theta = vel_theta - self.pose.theta
            #print("diff_theta before adjust")
            #print(diff_theta)

            #handle the excepetion where theta is an angle greater than pi or less than negative pi
            if(diff_theta > pi): 
                #print("diff_theta greater than pi")
                diff_theta = diff_theta/2 - pi
            elif(diff_theta < -pi):
                #print("diff_theta less than -pi")
                diff_theta = diff_theta/2 + pi

            #calculate the values of P_term from the PI Controller
            #print("diff_theta after adjust")
            #print(diff_theta)

            #change the values of Kp
            kp = 11    
            P_term = kp * diff_theta
           
            self.vel.angular.z = P_term
            self.vel_pub.publish(self.vel)
            self.rate.sleep()


        #update the previous waypoint and velocities for the contuinuity boundary
        self.previous_waypoint = current_waypoint
        print("Previous_waypoint_update: ")
        print(self.previous_waypoint)

        self.previous_velocity = [calc_vx, calc_vy]
	print("previous_velocity_x_end: ") 
        print(self.previous_velocity[0])
	print("previous_velocity_y_end: ")
        print(self.previous_velocity[1])

        if(count == length - 1):
            print("+++++++++++++++++++++++")
            self.vel.linear.x = 0.9
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

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

    def neighbors(self, current):
        #define the list of 4 neighbors
        neighbors = [[-1, 0], [0, 1], [1, 0], [0, -1]]
        return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

    def heuristic_distance(self, candidate, goal):
        #return the euclidean distance between the cnadiate and the goal
        return sqrt(pow((goal[0] - candidate[0]), 2) + pow((goal[1] - candidate[1]), 2))

    def get_path_from_A_star(self, start, goal, obstacles):
         # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
         #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
         #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
         # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
         #   note that the path should contain the goal but not the start
         #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 

         #creates an open list with the cost first and the point second
         open_list = [[0, start]]
    
         #create an empty closed list that is initialized as empty
         closed_list = []

         #create a dictionary for the past costs
         past_cost = {}
         past_cost[start] = 0

         #create a dictionary of parents
         parent = {}

         #initialize a path to return
         path = []

         #if the goal is in a obstacle return an error
         if goal in obstacles:
             print("ERROR: GOAL IS WITHIN AN OBSTACLE")
             return
    
         #while the open_list is not empty
         while open_list:
             #remove the current cheapest node from the open list and move it to closed 
             print(open_list)
             current = open_list.pop(0)
             closed_list.append(current[1])
        
             #if the current is within the goal the return, we dont want to do anything
             if current[1] == goal:
                 print("REACHED GOAL")
                 break

             for nbr in self.neighbors(current[1]):
                 if nbr not in closed_list and nbr not in obstacles: 
                     tentative_past_cost = past_cost[current[1]] + 1
                
                     #if the neighbor has not been previously visited we need to make a addition to the cost
                     if(nbr not in open_list):
                         past_cost[nbr] = tentative_past_cost

                     if tentative_past_cost <= past_cost[nbr]:
                         #update past cost with the lower value
                         past_cost[nbr] = tentative_past_cost
                         #set the parent of the previous nbr
                         parent[nbr] = current[1]
                         #calculate the estimated total cost based on past cost and the heuristic
                         est_total_cost = past_cost[nbr] + self.heuristic_distance(nbr, goal)
                         #append to open list and sort based on the total cost
                         open_list.append([est_total_cost, nbr])
                         open_list.sort()
                     #print(open_list)       
         #print(parent)
         #start from the goal to get path
         dict_index = goal 
         while(parent.get(dict_index) is not start):
             #append to front of path
             path.insert(0, parent[dict_index])
             dict_index = parent[dict_index]
      
         path.append(goal)
         return path

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
            #rospy.loginfo("odom: x=" + str(self.pose.x) +\
                #";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))     

if __name__ == '__main__':
    whatever = Turtlebot()

