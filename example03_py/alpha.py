# Source:
# https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files

# This function, alpha, calculates the rotation vector from a rotation matrix M.
# This rotation vector is a compact representation of a rotation in 3D space.


import numpy as np


def alpha(M):
    def I(M):
        return np.array([[M[2, 1] - M[1, 2]],
                         [M[0, 2] - M[2, 0]],
                         [M[1, 0] - M[0, 1]]])

    A = np.linalg.norm(I(M))  # if norm is 0 then M is a diagonal
    if A < 1e-6:  # instead of considering a diagonal with values of 0, we consider a very small value
        if M[0, 0] == 1 and M[1, 1] == 1 and M[2, 2] == 1:
            alpha = np.array([[0], [0], [0]])
        else:
            alpha = (np.pi / 2) * np.array([[M[0, 0] + 1],
                                            [M[1, 1] + 1],
                                            [M[2, 2] + 1]])
    else:  # not diagonal
        B = M[0, 0] + M[1, 1] + M[2, 2] - 1
        alpha = np.dot(np.divide(np.arctan2(A, B), A), I(M))

    return alpha
