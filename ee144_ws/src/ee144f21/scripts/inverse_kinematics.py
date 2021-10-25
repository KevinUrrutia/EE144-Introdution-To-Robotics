import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos, tan, asin

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    alpha = acos((pow(0.15,2) + pow(0.1581, 2) - pow(0.05, 2))/(2*(0.15)*(0.1581)))
    print(alpha)

    gamma = atan2((z-0.1039), (sqrt(pow(x,2) + pow(y,2))))
    print(gamma)
    
    C_1 = sqrt(pow(x,2) + pow(y,2) + pow((z - 0.1039), 2))
    print(C_1)

    beta_2 = acos((pow(0.1581,2) + pow(0.15, 2) - pow((C_1), 2))/(2*(0.15)*(0.1581)))
    #print(beta_2)

    beta_1 = acos((pow(0.1581,2) + pow(C_1, 2) - pow(0.15, 2))/(2*(C_1)*(0.1581)))
    print(beta_1)
    
    if (y >= 0):
        theta_2 = pi/2 - (alpha + gamma + beta_1)
    else:
        angle = gamma - (alpha + beta_1)
        theta_2 = angle - (pi/2)
    

    tau = (pi/2) - alpha
    theta_3 = tau + beta_2 - pi

    theta_1 = atan(y/x)

    joint1 = theta_1
    joint2 = theta_2
    joint3 = theta_3     

    return [joint1, joint2, joint3]


print(inverse_kinematics([-0.02590,-0.01495,0.35212]))
