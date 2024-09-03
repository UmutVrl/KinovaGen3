#  Source :
#  https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial/blob/main/PICK_PLACE/Python%20Pick%20and%20Place%20Files


# this file randomly guesses a pose as a transformation matrix
# the orienation is not random, only a random guess between 8 selected poses which are the most viable when picking up cubes
# the x y and z are random within the robots reach limitations

# The pose_guess method is a technique used to generate random initial poses for inverse kinematics calculations.
# By randomly selecting points within a semi-sphere and assigning orientations from a predefined set of matrices,
# this method provides diverse initial guesses for the end-effector’s position and orientation.
# The generated pose is represented as a 4x4 transformation matrix, ensuring that the initial guess falls within
# the specified constraints. This method is essential for initializing iterative optimization algorithms
# in achieving the desired end-effector pose.

import random
import numpy as np

def pose_guess():
    # Put all variables into a list
    while True:
        # Generate random points within a cube until one falls inside the semi-sphere
        radius = 0.73512
        max_z = 1.01793
        x = random.uniform(-radius, radius)
        y = random.uniform(-radius, radius)
        z = random.uniform(0, max_z)  # Limiting z to the upper hemisphere

        if (np.abs(x)>=0.15) and (np.abs(y)>=0.15) and  (x**2 + y**2 + z**2 <= radius**2):  # Check if the point is inside the semi-sphere
            
            A = np.array([[ 0,  1,  0 ],
                          [ 1,  0,  0 ],
                          [ 0,  0, -1 ]])
    
            B = np.array([[-1,  0,  0 ],
                          [ 0,  1,  0 ],
                          [ 0,  0, -1 ]])
    
            C = np.array([[-1,  0,  0 ],
                          [ 0,  0,  1 ],
                          [ 0,  1,  0 ]])
    
            D = np.array([[ 0,  0,  1 ],
                          [ 1,  0,  0 ],
                          [ 0,  1,  0 ]])
    
            E = np.array([[ 1,  0,  0 ],
                          [ 0,  0, -1 ],
                          [ 0,  1,  0 ]])
    
            F = np.array([[ 1,  0,  0 ],
                          [ 0, -1,  0 ],
                          [ 0,  0, -1 ]])
    
            G = np.array([[ 0,  0, -1 ],
                          [-1,  0,  0 ],
                          [ 0,  1,  0 ]])
    
            H = np.array([[ 0, -1,  0 ],
                          [-1,  0,  0 ],
                          [ 0,  0, -1 ]])
    
            variables = [A, B, C, D, E, F, G, H]
            rand_orien = random.choice(variables)
            pose_guess = np.eye(4)
            pose_guess[:3, :3] = rand_orien
            pose_guess[0,3] = x
            pose_guess[1,3] = y
            pose_guess[2,3] = z
            
            break 
    
    return pose_guess    
    
    

   
# print(pose_guess())

