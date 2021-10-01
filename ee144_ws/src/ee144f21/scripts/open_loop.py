#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class TurtleBot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()

    def run(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        i = 0
        while not rospy.is_shutdown():
            if i is not 4:
                vel.linear.x += 0.4
                vel.angular.z += 90
                i += 1
                rospy.spin(10)
            else:
                vel.linear.x = 0
                vel.angular.z = 0
                val = raw_input("Finished rotation continue y/n?")
                if val == 'y':
                    i = 0
                elif val == 'n':
                    break

if __name__ == '__main__':
    try:
        whatever = TurtleBot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated")
