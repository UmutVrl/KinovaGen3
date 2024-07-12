#  Source :
#  https://colab.research.google.com/drive/1pJDImFs-ytFj88h5XOlPONxh1ScuNfLG#scrollTo=_bv8ODYZ4U52
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files
#
# This method is a constrained optimization method that finds the minimum of a scalar function of one or more
# variables subject to constraints.
#
# trust_constr method takes an initial guess of joint angles, a target transformation matrix Tt, and a number
# of iterations as inputs. The function calculates the transformation matrices for each joint and the end effector,
# and defines an objective function that calculates the total error, which is a combination of pose error and z error.
#
# The pose error is the norm of the difference between the target transformation matrix and the forward kinematics of
# the current joint angles. The z error is calculated for each joint and the end effector, and is based on the
# z-coordinate of the joint or end effector position.
#
# The function then uses the minimize function with the trust-constr method to find the joint angles that minimize the
# total error. The function returns whether the optimization was successful, the optimal joint angles in degrees,
# and the number of iterations used.
#

import numpy as np
from scipy.optimize import minimize
from Rx import Rx
from Rz import Rz
from fk import fk
#from pose_guess import pose_guess
#from QuadraticError import QuadraticError
#from angle_guess import angle_guess
from T import T
from Err import Err


def trust_constr(initial_guess, Tt, iterations):
    # Define the fixed transformation matrices outside the optimization loop
    E1 = T(0, 0, 0.15643)
    E2 = Rx(np.pi)
    ETS1 = np.dot(E1, E2)
    E4 = T(0, 0, -0.12838)
    E5 = T(0, 0.00538, 0)
    E6 = Rx(np.pi / 2)
    ETS2 = np.linalg.multi_dot([E4, E5, E6])
    E8 = T(0, 0, -0.00638)
    E9 = T(0, -0.21038, 0)
    E10 = Rx(-np.pi / 2)
    ETS3 = np.linalg.multi_dot([E8, E9, E10])
    E12 = T(0, 0, -0.21038)
    E13 = T(0, 0.00638, 0)
    E14 = Rx(np.pi / 2)
    ETS4 = np.linalg.multi_dot([E12, E13, E14])
    E16 = T(0, 0, -0.00638)
    E17 = T(0, -0.20843, 0)
    E18 = Rx(-np.pi / 2)
    ETS5 = np.linalg.multi_dot([E16, E17, E18])
    E20 = T(0, 0, -0.10593)
    E21 = Rx(np.pi / 2)
    ETS6 = np.dot(E20, E21)
    E23 = T(0, -0.10593, 0)
    E24 = Rx(-np.pi / 2)
    ETS7 = np.dot(E23, E24)
    E26 = T(0, 0, -0.06153)
    E27 = Rx(np.pi)
    ETS8 = np.dot(E26, E27)

    def objective(x):
        Joint4_Matrix = np.linalg.multi_dot(
            [ETS1, Rz(x[0]), ETS2, Rz(x[1]), ETS3, Rz(x[2]), ETS4, Rz(x[3]), ETS5, Rz(x[4])])
        Joint5_Matrix = np.linalg.multi_dot(
            [ETS1, Rz(x[0]), ETS2, Rz(x[1]), ETS3, Rz(x[2]), ETS4, Rz(x[3]), ETS5, Rz(x[4]), ETS6, Rz(x[5])])
        Joint6_Matrix = np.linalg.multi_dot(
            [ETS1, Rz(x[0]), ETS2, Rz(x[1]), ETS3, Rz(x[2]), ETS4, Rz(x[3]), ETS5, Rz(x[4]), ETS6, Rz(x[5]), ETS7,
             Rz(x[6])])
        EndEffector_Matrix = np.linalg.multi_dot(
            [ETS1, Rz(x[0]), ETS2, Rz(x[1]), ETS3, Rz(x[2]), ETS4, Rz(x[3]), ETS5, Rz(x[4]), ETS6, Rz(x[5]), ETS7,
             Rz(x[6]), ETS8])

        a = 1e-100
        jz4 = (1 / (np.abs(a) * np.sqrt(np.pi))) * np.exp(-(np.square((Joint4_Matrix[2, 3] - 0.02) / a)))
        jz5 = (1 / (np.abs(a) * np.sqrt(np.pi))) * np.exp(-(np.square((Joint5_Matrix[2, 3] - 0.02) / a)))
        jz6 = (1 / (np.abs(a) * np.sqrt(np.pi))) * np.exp(-(np.square((Joint6_Matrix[2, 3] - 0.02) / a)))
        jz7 = (1 / (np.abs(a) * np.sqrt(np.pi))) * np.exp(-(np.square((EndEffector_Matrix[2, 3] - 0.04) / a)))
        z_error = jz4 + jz5 + jz6 + jz7
        pose_error = np.linalg.norm(Err(Tt, fk(x)))

        # Combine both errors with a weighted sum
        total_error = pose_error + z_error

        return total_error

    # Define constraints
    lb = np.deg2rad([-360, -128.9, -360, -147.8, -360, -120.3, -360])
    ub = np.deg2rad([360, 128.9, 360, 147.8, 360, 120.3, 360])

    # Initializations
    d = np.empty(7)

    # Perform optimization
    result = minimize(
        objective, initial_guess,
        bounds=list(zip(lb, ub)),
        method='trust-constr',
        jac='2-point',  # You can experiment with '2-point' or '3-point' finite differences
        hessp=None,  # Set to None for now, you can experiment with other options
        options={'maxiter': iterations, 'xtol': 0.02},
    )

    # Display results
    if result.success:
        print("Optimization successful.")
        print("Number of iterations:", result.nit)
        print("Optimal Joint Angles:", np.rad2deg(result.x))
        print("Optimal Desired Matrix:")
        print(fk(result.x))
        print(f'Trust-Constr method converged in {result.nit} iterations.')
        converged = 1
    else:
        print("Optimization did not converge.")
        result.nit = []
        result.x = []
        converged = 0

    return converged, np.rad2deg(result.x), result.nit


#T1 = pose_guess()
#q1 = angle_guess()
#angles2 = trust_constr(q1,T1)