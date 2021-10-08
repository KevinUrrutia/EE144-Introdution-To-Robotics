#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time
from math import pi
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion

class TurtleBot():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
	sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.Position)
        self.rate = rospy.Rate(10) 
        self.run()
      
    def Position(self, msg):
        print msg.pose

    def run(self):
        vel = Twist()
        
        i = 0
        while not rospy.is_shutdown():
            if i < 4:      
                for j in range(80):
                    vel.linear.x = 0.5
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    self.rate.sleep()
                #time_end = time.time() + 14
	        #while time.time() < time_end:
                    #vel.linear.x = 0.5
                    #vel.angular.z = 0
                    #self.vel_pub.publish(vel)
                    #self.rate.sleep()

      	        time.sleep(1)
	
<<<<<<< HEAD
                time_end = time.time() + 3.2
                while time.time() < time_end:
=======
		for j in range(18):
>>>>>>> 331bafc0006e75748fb472f2d95e0fdb936dd080
                    vel.linear.x = 0
                    vel.angular.z = pi/4
                    self.vel_pub.publish(vel)
                    self.rate.sleep()
                #time_end = time.time() + 3
                #while time.time() < time_end:
                    #vel.linear.x = 0
                    #vel.angular.z = pi/4
                    #self.vel_pub.publish(vel)
                    #self.rate.sleep()  
         
                time.sleep(1)  
                i += 1
            else:
                break

            
if __name__ == '__main__':
    try:
        whatever = TurtleBot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated")
