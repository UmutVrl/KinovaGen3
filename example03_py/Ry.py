# see: https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing#scrollTo=nwKBVppDkHKk

import numpy as np


def Ry(theta):
    """Rotation matrix y-axis
            inputs a rotation angle around the y-axis and returns the rotation matrix
        Arguments:
            theta
            return Ry_matrix
    """
    Ry_matrix = np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])
    return Ry_matrix