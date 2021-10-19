#!/usr/bin/env python

import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    #input: joint angles [joint1, joint2, joint3]
    #output: the position of the end effector [x,y,z]
    #add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150

    joint1 = joint[0]
    joint2 = joint[1]
    joint3 = joint[2]
  
    #Find the Home Postion of the robotic arm
    M = np.matrix([1,0,0,link3x + link4x],[0,1,0,0],[0,0,1,link1z + link2z + link3z], [0,0,0,1])
    
    #Screw axis for joint 3	
    S3 = np.matrix([0],[1],[0], [-0.254], [0], [0.05])
    bracket_S3 = mr.VecTose3(S3) * joint3
    exp_S3 = mr.MatrixExp6(bracket_S3)

    #Screw axis for joint 2	
    S2 = np.matrix([0],[1],[0], [-0.104], [0], [0])
    bracket_S2 = mr.VecTose3(S2) * joint2
    exp_S2 = mr.MatrixExp6(bracket_S2)
   
    #Screw axis for joint 1	
    S1 = np.matrix([0],[1],[0], [0], [0], [0])
    bracket_S1 = mr.VecTose3(S1) * joint1
    exp_S1 = mr.MatrixExp6(bracket_S1)

    EOF = exp_S1 * exp_S2 * exp_S3 * M
    x = EOF[0][3]
    y = EOF[1][3]
    z = EOF[2][3]

    #Pull out x,y,z from matrix

    return [x, y, z]
