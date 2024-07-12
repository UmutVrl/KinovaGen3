# see: https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing#scrollTo=nwKBVppDkHKk

import numpy as np

def T(dx, dy, dz):
    """Translation Matrix
            combines all the individual translation matrices into a singular function
    Arguments:
         dx, dy, dz
         return T_matrix
    """
    T_matrix = np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])
    return T_matrix