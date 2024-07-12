#  Source :
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files

# This function, Err, calculates the error between two transformation matrices, Ted and Tec.
# The error is calculated in terms of both position and orientation.


import numpy as np
from alpha import alpha


def Err(Ted, Tec):
    Err = np.vstack([(Ted[:-1, -1] - Tec[:-1, -1]).reshape(-1, 1),  # transforms into a 3by1
                 alpha(np.dot(Ted[:3, :3], np.transpose(Tec[:3, :3])))
                ])
    return Err