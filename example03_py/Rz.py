# see: https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing#scrollTo=nwKBVppDkHKk

import numpy as np

def Rz(theta):
    """Rotation matrix for z-axis
            inputs a rotation angle around the z axis and returns the rotation matrix
        Arguments:
            theta
            return Rz_matrix
    """
    Rz_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return Rz_matrix