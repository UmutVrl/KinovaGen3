#  Source :
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files


# this function inputs 2 transformation matrices and returns mean square error between them
# line 7 is the weights for each value in the error vector ( see tutorial)

# The Quadratic Error method is a technique used to quantify the discrepancy between the desired and
# current transformation matrices in inverse kinematics calculations. By computing the weighted sum
# of squared errors, this method provides a measure of how closely the current pose of the end-effector
# matches the target pose. The process involves calculating the error vector, applying a weight matrix,
# and performing matrix multiplications to obtain the quadratic error. This method is essential for
# evaluating the performance of iterative optimization algorithms in achieving the desired end-effector
# position and orientation.

import numpy as np
from Err import Err

def QuadraticError(Ted, Tec):
    we = np.array([1, 1, 1, 1, 1, 1])
    We = np.diag(we)
    E = 0.5 * np.transpose(Err(Ted, Tec)).dot(We).dot(Err(Ted, Tec))
    return E
