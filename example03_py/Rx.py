# see: https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing#scrollTo=nwKBVppDkHKk

import numpy as np


def Rx(theta):
    """Rotation matrix x-axis
            inputs a rotation angle around the x-axis and returns the rotation matrix
        Arguments:
            theta
            return Rx_matrix
    """
    Rx_matrix = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])
    return Rx_matrix
