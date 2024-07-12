#  Source :
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files

# This method generates a random initial guess for the joint angles of a 7 degree-of-freedom robot.

import numpy as np


def angle_guess():
    # Define the bounds
    max_val = 360
    min_val = -360
    max_q2 = 128.9
    min_q2 = -128.9
    max_q4 = 147.8
    min_q4 = -147.8
    max_q6 = 120.3
    min_q6 = -120.3

    # Initial guesses for joint angles
    q1 = np.random.uniform(min_val, max_val)
    q2 = np.random.uniform(min_q2, max_q2)
    q3 = np.random.uniform(min_val, max_val)
    q4 = np.random.uniform(min_q4, max_q4)
    q5 = np.random.uniform(min_val, max_val)
    q6 = np.random.uniform(min_q6, max_q6)
    q7 = np.random.uniform(min_val, max_val)

    return np.deg2rad([q1, q2, q3, q4, q5, q6, q7])
