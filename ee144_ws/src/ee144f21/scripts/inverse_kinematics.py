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

    alpha = atan2(link3x, link3z)
    #print(alpha)
    gamma = atan2(z + 0.1039, x) 
    #print(gamma)
    
    b_length1 = sqrt((link3x ** 2) + (link3z ** 2))
    print(b_length1)
    c_length1 = z / (sin(gamma))
    print(c_length1)
    a_length1 = link4x
    print(a_length1)
    beta_2 = acos(((c_length1 ** 2) - (a_length1 **2) - (b_length1 ** 2)) /(-2* a_length1 * b_length1))
    print(beta_2)
 
    #a_length2 = b_length1
    #print(a_length2)
    #b_length2 = c_length1
    #print(b_length2)
    #c_length2 = a_length1
    #c_length_2_sq = c_length2 ** 2
    #print(c_length_2_sq)
    #a_length_2_sq = a_length2 ** 2
    #print(a_length_2_sq)
    #b_length_2_sq = b_length2 ** 2
    #print(b_length_2_sq)
    #beta_1 = acos(((c_length2 ** 2) - (a_length2 **2) - (b_length2 ** 2)) /(-2 * a_length2 * b_length2))

    theta_2 = 90 - alpha - gamma - beta_1

    tau = 90 - alpha
    theta_3 = tau + beta_2 - 180

    theta_1 = atan2(y, x)

    joint1 = theta_1
    joint2 = theta_2
    joint3 = theta_3     

    return [joint1, joint2, joint3]


print(inverse_kinematics([-0.02590,-0.01495,0.35220]))
