#  Source :
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files

# These functions, smallest_angular_distance_limits and smallest_angular_distance_nolimits,
# calculate the smallest angular distance between two angles.
#
# The function smallest_angular_distance_limits takes into account the limits of the angles.
# It checks if both angles are in the top half or the bottom half of the unit circle, or if one is in the top half and
# the other is in the bottom half. Depending on these conditions, it calculates the clockwise (cw) and counter-clockwise
# (ccw) distances between the angles and returns the smallest one. If the angles are not clearly in the top or
# bottom half, it returns a message indicating this.
#
# The function smallest_angular_distance_nolimits does not consider any limits. It simply calculates the clockwise and
# counter-clockwise distances between the angles and returns the smallest one.
#
# Both functions first normalize the input angles to the range [0, 360) degrees using the modulo operation.
# The distances are calculated using the formula (angle1 - angle2 + 360) % 360, which gives the distance when moving
# from angle1 to angle2 in the specified direction (clockwise or counter-clockwise). The np.abs function is used to get
# the absolute value of the distance, and the smaller distance is returned.
#
# These functions can be useful in robotics and other fields where it’s necessary to calculate the smallest rotation
# between two orientations.


import numpy as np
import math


def smallest_angular_distance_limits(initial_angle, final_angle):
    initial_angle = (initial_angle + 360) % 360
    final_angle = (final_angle + 360) % 360
    # Check if both angles are in the top half
    if (math.sin(np.deg2rad(initial_angle)) >= 0) and (math.sin(np.deg2rad(final_angle)) > 0):
        cw = np.abs((initial_angle - final_angle + 360) % 360)
        ccw = np.abs((final_angle - initial_angle + 360) % 360)
        if ccw < cw:
            return ccw
        else:
            return -cw

    # Check if both angles are in the bottom half
    elif (math.sin(np.deg2rad(initial_angle)) <= 0) and (math.sin(np.deg2rad(final_angle)) < 0):
        cw = np.abs((initial_angle - final_angle + 360) % 360)
        ccw = np.abs((final_angle - initial_angle + 360) % 360)
        if ccw < cw:
            return ccw
        else:
            return -cw

    # Check if initial angle is in the top half and final angle is in the bottom half
    elif (math.sin(np.deg2rad(initial_angle)) >= 0) and (math.sin(np.deg2rad(final_angle)) < 0):
        return -np.abs((initial_angle - final_angle + 360) % 360)

    # Check if initial angle is in the bottom half and final angle is in the top half
    elif (math.sin(np.deg2rad(initial_angle)) <= 0) and (math.sin(np.deg2rad(final_angle)) > 0):
        return np.abs((final_angle - initial_angle + 360) % 360)

    # If none of the above conditions are met
    else:
        return "Angles are not clearly in the top or bottom half."


def smallest_angular_distance_nolimits(initial_angle, final_angle):
    initial_angle = (initial_angle + 360) % 360
    final_angle = (final_angle + 360) % 360
    cw = np.abs((initial_angle - final_angle + 360) % 360)
    ccw = np.abs((final_angle - initial_angle + 360) % 360)
    if ccw < cw:
        return ccw
    else:
        return -cw
