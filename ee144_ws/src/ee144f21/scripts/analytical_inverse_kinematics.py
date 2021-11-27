import numpy as np

def inverse_kinematics(position):
    x = position[0]
    y = position[1]
    z = position[2]

    joints = [x, y, z]

    return joints

test = np.array([0.3, 0.3, 0.3, 0])
print(inverse_kinematics(test))
