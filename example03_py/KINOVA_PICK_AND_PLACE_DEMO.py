#######################################################################################################
#    Pick and Place Example for Kinova Gen3 Robotic Arm                                               #
#    written by: U. Vural                                                                             #
#                                                                                                     #
#    for KISS Project at Furtwangen University                                                        #
#                                                                                                     #
#    This program is simplified & modified version of University of Calgary ENME 501/502 Group        #
#    Pick & Place example. Please check the original work that cited below.                           #
#                                                                                                     #
#    This Demo runs with Large cube and Base Aruco Marker. It detects the large cube and place it to  #
#    detected base location. Put first item then wait for calculations by monitoring the log and put  #
#    second one. The locations should be not too near to each other and edges. Expect glitches so run #
#    with caution and ready to press emergency halt. See sample operation video in readme section.    #
#                                                                                                     #
#                                                                                                     #
#    Source:                                                                                          #
#    Design and Implementation of a Pick and Place Controller for the Kinova Gen 3 Arm                #
#    (University of Calgary ENME 501/502 Group 23)                                                    #
#    https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial           #
#                                                                                                     #
#    # See Readme for Warnings and Notes                                                              #
#                                                                                                     #
#######################################################################################################
#    specs: (SEE Readme for more)                                                                     #
#    Python 3.9                                                                                       #
#    Kinova Kortex 2.6.0                                                                              #
#    Gen3 firmware Bundle 2.5.2-r.2                                                                   #
#    opencv-python 4.5.*                                                                              #
#    opencv-contrib-python 4.5.*                                                                      #
#    pandas 2.2.1                                                                                     #
#    matplotlib 3.8.1                                                                                 #
#    scipy 1.12.0                                                                                     #
#                                                                                                     #
#######################################################################################################

# this is the complete pick and place controller file,
# make sure all the vision_sensor_focus_action_filepath lines have the correct path to the focus350 file
import datetime
import multiprocessing
import subprocess
import utilities
import intrinsics
import cv2
import cv2.aruco as aruco
from smallest_angular_distance import smallest_angular_distance_limits
from smallest_angular_distance import smallest_angular_distance_nolimits
from angle_guess import angle_guess
from trust_constr import trust_constr
from Rx import Rx
from Ry import Ry
from Rz import Rz
from T import T
from SM import SM
from CM import CM
from NR import NR
from WM import WM
from GN import GN
from fk import fk
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import (Session_pb2, Base_pb2, DeviceConfig_pb2,
                                         Session_pb2, DeviceManager_pb2, VisionConfig_pb2, BaseCyclic_pb2, Common_pb2)
import sys
import os
import time
import threading
import traceback
import numpy as np
from close_gripper import close_gripper
from get_angles_pose import get_angles_pose

TIMEOUT_DURATION = 10000
global_return_values = []
BASE01_POS_Z = 0.05  # (meters)

# Calibration for 1280x720
# TO DO: Use Screenshot_Taker_timer.py and Camera_Calibration.py to get this values. See Readme
# 27.05.24 1280x720
cameraMatrix = np.array([[1.27981674e+03, 0.00000000e+00, 6.37487808e+02],
                         [0.00000000e+00, 1.27830601e+03, 3.00103130e+02],
                         [0, 0, 1]], dtype=np.float32)
distCoeffs = np.array([-0.00535619,  0.0999779 ,  0.01491466, -0.00593549, -1.00733407], dtype=np.float32)


# TO DO: Check file PATH
vision_sensor_focus_action_filepath = os.getcwd() + r"\focus_350.py"
print(vision_sensor_focus_action_filepath)

def move_to_home_position(base):
    """ Abstract: Moves arm to a ‘Home’ position. Sets the servoing mode, finds and executes the ‘Home’ action, and
        handles errors. Provides real-time feedback and returns a boolean indicating the success of the operation.

        Parameter:
        :param base: Base of the robotic arm which the function operates on.
        :return:
    """
    # print("MOVE_TO_HOME_POSITION IS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # print(cv2.getBuildInformation())
    # Make  sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    # print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)

    finished = e.wait(TIMEOUT_DURATION)
    pose = base.GetMeasuredCartesianPose()
    camera_pose = camera_coor(pose)
    base.Unsubscribe(notification_handle)
    #print("Home_Position camera_pose: ", camera_pose)
    #print("Home_Position cartesian_pose: ", pose)

    if finished:
        print("MOVE_TO_HOME_POSITION IS COMPLETED...")
        #print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    # print("CHECK_FOR_END_OR_ABORT IS CALLED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def look_position():
    """
    Abstract: Moves arm to a specific position, defined by a set of joint angles. Moves the arm to its home position,
    then executes an action to reach the desired joint angles. Waits for the action to finish or for a timeout to occur.
    Then retrieves and prints the current pose of the arm in both Cartesian and camera coordinates. Returns a boolean
    indicating whether the action was completed successfully.

    :return:
    """
    # print("LOOK_POSITION IS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
    #import utilities

    args = utilities.parseConnectionArguments()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        move_to_home_position(base)
        #print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = base.GetActuatorCount()

        # Place arm straight up
        i = 0
        # Actuator #1, #2, #3, #4, #5, #6, #7
        #angle_values = [360, 295, 180, 235, 0, 270, 87]
        angle_values = [357, 300, 183, 247, 3, 271, 88]
        # angle_values = [360, 15, 180, 280, 0, 270, 70]
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = angle_values[i]
            i = i + 1

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        #print("Executing action")
        base.ExecuteAction(action)

        #print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)
        pose = base.GetMeasuredCartesianPose()
        camera_pose = camera_coor(pose)

        if finished:
            print("LOOK_POSITION camera_pose: ", camera_pose)
            print("LOOK_POSITION cartesian_pose: ", pose)
            print("LOOK_POSITION IS FINISHED...")
            #print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished


def spin(switch):
    """
    Abstract: Controls the spinning motion of a robotic arm with 7 actuators. Depending on the value of the input
    switch, either spins related joints of the arm, pauses the arm, or stops the function. Alternates direction every
    0.3 seconds. The function provides real-time feedback and stops all joint movements immediately when the spinning
    process is completed.

    :param switch: Input switch that controls the behavior of the function. If the value is 0, the function stops
    If the value is 1, the function spins the joints of the arm. If the value is 2, the function pauses the arm.
    :return:
    """
    #print("SPIN PROCESS IS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        actuator_count = base.GetActuatorCount().count

        if actuator_count == 7:
            speed6 = 0  # Initial speed for the 6th joint
            while True:  # Run indefinitely until explicitly stopped
                if switch.value == 0:  # Check if the switch is off
                    #print("Spin stopped.")
                    break  # Exit the loop to stop

                elif switch.value == 1:  # Spin with specified speeds
                    #print("Spinning...")
                    joint_speeds = Base_pb2.JointSpeeds()  # Recreate joint_speeds each iteration
                    # Actuator #1, #2, #3, #4, #5, #6, #7
                    speeds = [0, 0, 0, 0, 0, 0, speed6]  # Update speeds list with the current speed6

                    for i, speed in enumerate(speeds):
                        joint_speed = joint_speeds.joint_speeds.add()
                        joint_speed.joint_identifier = i
                        joint_speed.value = speed
                        joint_speed.duration = 0

                    base.SendJointSpeedsCommand(joint_speeds)

                    # Flip the speed6 for the next iteration
                    speed6 = -speed6
                    time.sleep(0.3)

                elif switch.value == 2:  # Pause with 0 speeds
                    #print('Paused')
                    joint_speeds = Base_pb2.JointSpeeds()  # Recreate joint_speeds each iteration
                    speeds = [0, 0, 0, 0, 0, 0, 0]  # Zero speeds

                    for i, speed in enumerate(speeds):
                        joint_speed = joint_speeds.joint_speeds.add()
                        joint_speed.joint_identifier = i
                        joint_speed.value = speed
                        joint_speed.duration = 0

                    base.SendJointSpeedsCommand(joint_speeds)

            base.Stop()  # Stop the joint movements immediately
            print("SPIN PROCESS IS COMPLETED...")
            # return True


def camera_coor(pose):
    """
    Abstract: The camera_coor function calculates the transformation matrix for a camera attached to a robotic arm.
    It takes the pose of the arm as input, which includes the position (x, y, z) and orientation (theta_x, theta_y,
    theta_z). The function first calculates the rotation matrix R using the orientation angles. It then creates a 4x4
    transformation matrix M with R and the position. Finally, it calculates the transformation matrix C for the camera
    by multiplying M with a translation matrix T that represents the distance between the end effector and the camera.
    The function returns C, the transformation matrix for the camera.

    :param pose: This is the pose of the robotic arm. It includes the position (x, y, z) and orientation (theta_x,
    theta_y, theta_z) of the arm. The exact type and structure of pose would depend on the specific robotic arm system
    being used and the overall structure of the codebase this function is part of. It’s likely to be an instance of a
    class or a data structure that represents the pose of the robotic arm.
    :return:
    """
    # print("CAMERA_COOR METHOD IS CALLED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    x = pose.x
    y = pose.y
    z = pose.z
    X = pose.theta_x
    Y = pose.theta_y
    Z = pose.theta_z
    rx = Rx(np.deg2rad(X))
    ry = Ry(np.deg2rad(Y))
    rz = Rz(np.deg2rad(Z))
    terms1 = [rz, ry, rx]
    R = np.linalg.multi_dot(terms1)
    M = np.eye(4)
    M[:3, :3] = R[:3, :3]
    M[:3, 3] = [x, y, z]
    terms2 = [M, T(0, 0.062, -0.151)] # distance between EE and CAM in meters
    # terms2 = [M, T(0, 0.054, -0.138)] # CALIBRATE & MEASURE
    # terms2= [M,T(0, 0.054, -0.124)]
    C = np.linalg.multi_dot(terms2)
    return C


def controller_vision_find(switch, global_return_values, coordinates):
    """
    Abstract: The controller_vision_find function is designed to control a vision system that identifies and tracks
    markers in a video stream. It uses the ArUco library to detect markers in each frame, estimate their pose, and draw
    them on the frame. Depending on the ID of the detected marker, the function updates a list of return values and a
    switch value. The function runs indefinitely until ‘q’ is pressed or an error occurs. It provides real-time feedback
    and releases all resources when the process is stopped.

    :param switch: This is an input switch that controls the behavior of the function. Depending on the detected marker
                        IDs and the current state of the system, the function updates the value of the switch.
    :param global_return_values: This is a list that stores the return values corresponding to the detected marker IDs.
                        The function updates this list as it detects new markers.
    :param coordinates: This is a dictionary that stores the coordinates of the detected markers.
    :return:
    """
    # print("CONTROLLER_VISION_FIND IS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Run the focus action script to adjust the camera before starting the capture
    subprocess.run(['python', vision_sensor_focus_action_filepath])

    cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return 1  # Indicate an error occurred

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    #frame_width = int(cap.get(3))
    #frame_height = int(cap.get(4))
    #frame_size = (frame_width, frame_height)

    #result03 = cv2.VideoWriter('filename03.avi',
    #                         cv2.VideoWriter_fourcc(*'MJPG'),
    #                         10, frame_size)

    while True:  # Infinite loop
        ret, frame = cap.read()
        frame = cv2.resize(frame, (frame.shape[1], frame.shape[0]))
        if not ret:
            print("Error: Cannot read frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05,
                                                                       cameraMatrix, distCoeffs)
            for i in range(ids.size):
                aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.01)

            for id_array in ids:
                first_id = id_array[0]  # Get the first detected ID
                # Map the detected ID to the desired return value
                if 1 <= first_id <= 6:
                    return_value = 1  # LARGE CUBE
                # MODIFICATION TO JUST BASE AND LARGE CUBE
                #elif 7 <= first_id <= 12:
                #    return_value = 2  # MEDIUM CUBE
                #elif 13 <= first_id <= 18:
                #    return_value = 1  # SMALL CUBE
                elif first_id == 0:
                    return_value = 2  # BASE
                else:
                    return_value = 0  # Default return value if ID does not match any case

                if return_value not in global_return_values:
                    global_return_values.append(return_value)
                    switch.value = 2  # Notify about the new value (pause spin)

                    print(f"New value {return_value} added, switching to pause. Switch value now: {switch.value}")

                # MODIFICATION TO JUST BASE AND LARGE CUBE
                # Check if all required values are present and turn off if so
                if all(x in global_return_values for x in [1, 2]) and all(
                        coordinates.get(key) is not None for key in ["base", "large"]):
                    # print("All values detected. Stopping spin.")
                    switch.value = 0  # Turn off the switch

                #if all(x in global_return_values for x in [1, 2, 3, 4]) and all(
                #        coordinates.get(key) is not None for key in ["base", "small", "medium", "large"]):
                #    # print("All values detected. Stopping spin.")
                #    switch.value = 0  # Turn off the switch

        # Display the frame with detected markers
        #frame = cv2.resize(frame, (frame.shape[1], frame.shape[0]))
        #print(f"width: {frame.shape[1]}, height: {frame.shape[0]}")
        #result03.write(frame)
        cv2.imshow('Camera Feed', frame)
        # while switch.value == 2:  # Check if the switch is off
        #     print('pause')
        #     time.sleep(2)
        #     switch.value = 1
        #     break
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break the loop if 'q' is pressed
            break

        # Release resources if exited the loop without detecting any marker
    cap.release()
    cv2.destroyAllWindows()
    print("CONTROLLER_VISION_FIND PROCESS STOPPED...")
    return 0


def EOP(switch, coordinates):
    """
    Abstract: The EOP function is designed to control a vision system that identifies and tracks markers in a video
    stream. It uses the ArUco library to detect markers in each frame, estimate their pose, and draw them on the frame.
    Depending on the ID of the detected marker, the function updates a dictionary of coordinates and a switch value. The
    function runs indefinitely until the switch value is 0 or an error occurs. It provides real-time feedback and
    releases all resources when the process is stopped.

    :param switch: This is an input switch that controls the behavior of the function. Depending on the detected marker
                        IDs and the current state of the system, the function updates the value of the switch.
    :param coordinates: This is a dictionary that stores the coordinates of the detected markers.
    :return:
    """
    #print("EOP PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
    import utilities

    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)

        while switch.value != 0:
            if switch.value == 2:

                cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

                if not cap.isOpened():
                    print("Error: Could not open camera.")
                    return

                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                last_print_time = time.time()
                ret, frame = cap.read()

                if not ret:
                    print("Error: Cannot read frame.")
                    break

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        # Determine the marker size and category
                        if marker_id in range(1, 7) and coordinates["large"] is None:

                            subprocess.run(['python', vision_sensor_focus_action_filepath])

                            #size = 0.0535
                            size = 0.047
                            # corners: vector of marker corners
                            # size: marker siz ein meters
                            # cameraMatrix and distCoeffs: camera Calibration parameters
                            # rvecs and tvecs: rotation and translation ve
                            s = 0.06
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size,
                                                                                       cameraMatrix, distCoeffs)

                            time.sleep(2)
                            pose = base.GetMeasuredCartesianPose()
                            camera_pose = camera_coor(pose)
                            print("Large: EE Cartesian_Pos:", pose)
                            print("Large: Camera_Pos", camera_pose)
                            rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                            print("Large: ROT_M: ", rot_m)
                            terms4 = [camera_pose, rot_m]
                            print("Large: Terms_4: ", terms4)
                            Aruco_matrix = np.linalg.multi_dot(terms4)
                            print("Large: Aruco_matrix", Aruco_matrix)
                            A = np.dot(Aruco_matrix, T(0, 0, (s / 2)))
                            coordinates["large"] = A
                            print("Large: Coordinates", A)
                            if coordinates["large"] is not None:
                                #print(coordinates)
                                switch.value = 1
                                #print("CONTINUE SPINNING...")
                        if marker_id in range(7, 13) and coordinates["medium"] is None:

                            subprocess.run(['python', vision_sensor_focus_action_filepath])
                            #size = 0.04
                            size = 0.038
                            s = 0.046
                            # corners: vector of marker corners
                            # size: marker siz ein meters
                            # cameraMatrix and distCoeffs: camera Calibration parameters
                            # rvecs and tvecs: rotation and translation ve
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size,
                                                                                       cameraMatrix, distCoeffs)
                            time.sleep(2)
                            pose = base.GetMeasuredCartesianPose()
                            camera_pose = camera_coor(pose)
                            print("Med: EE Cartesian_Pos:", pose)
                            print("Med: Camera_Pos", camera_pose)
                            rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                            print("Med: ROT_M: ", rot_m)
                            terms4 = [camera_pose, rot_m]
                            print("Med: Terms_4: ", terms4)
                            Aruco_matrix = np.linalg.multi_dot(terms4)
                            print("Med: Aruco_matrix", Aruco_matrix)
                            A = np.dot(Aruco_matrix, T(0, 0, (s / 2)))
                            coordinates["medium"] = A
                            print("Med Coordinates",  A)
                            if coordinates["medium"] is not None:
                                #print(coordinates)
                                switch.value = 1
                                #print("CONTINUE SPINNING...")
                        if marker_id in range(13, 19) and coordinates["small"] is None:

                            subprocess.run(['python', vision_sensor_focus_action_filepath])
                            #size = 0.027
                            size = 0.025
                            s = 0.03
                            # corners: vector of marker corners
                            # size: marker siz ein meters
                            # cameraMatrix and distCoeffs: camera Calibration parameters
                            # rvecs and tvecs: rotation and translation vectors

                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size,
                                                                                       cameraMatrix, distCoeffs)
                            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 0.03)
                            time.sleep(2)
                            pose = base.GetMeasuredCartesianPose()
                            camera_pose = camera_coor(pose)
                            print("Small: EE Cartesian_Pos:", pose)
                            print("Small: Camera_Pos", camera_pose)
                            rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                            print("Small: ROT_M: ", rot_m)
                            terms4 = [camera_pose, rot_m]
                            print("Small: Terms_4: ", terms4)
                            Aruco_matrix = np.linalg.multi_dot(terms4)
                            print("Small: Aruco_matrix", Aruco_matrix)
                            A = np.dot(Aruco_matrix, T(0, 0, (s / 2)))
                            coordinates["small"] = A
                            print("Small Coordinates", A)
                            if coordinates["small"] is not None:
                                #print(coordinates)
                                switch.value = 1
                                #print("CONTINUE SPINNING...")
                        if marker_id == 0 and coordinates["base"] is None:

                            subprocess.run(['python', vision_sensor_focus_action_filepath])

                            # size = 0.0535
                            size = 0.0515
                            s = 0.06
                            # corners: vector of marker corners
                            # size: marker siz ein meters
                            # cameraMatrix and distCoeffs: camera Calibration parameters
                            # rvecs and tvecs: rotation and translation ve
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size,
                                                                                       cameraMatrix, distCoeffs)
                            time.sleep(10)
                            print(f"tvecs: {tvecs}, \n rvecs: {rvecs}, \n, _objPoints: {_objPoints}")
                            pose = base.GetMeasuredCartesianPose()
                            camera_pose = camera_coor(pose)
                            print("Base: EE Cartesian_Pos:", pose)
                            print("Base: Camera_Pos", camera_pose)
                            rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                            #rot_m = T(tvecs[0][0][2], -tvecs[0][0][1], -tvecs[0][0][0])
                            print("Base: ROT_M: ", rot_m)
                            terms4 = [camera_pose, rot_m]
                            print("Base: Terms_4: ", terms4)
                            Aruco_matrix = np.linalg.multi_dot(terms4)
                            print("Base: Aruco_matrix", Aruco_matrix)
                            A = np.dot(Aruco_matrix, T(0, 0, (s / 2)))
                            #A = np.array([[-0.01289517, 0.99938621, 0.03257164, 0.39326723],
                            #              [0.99807967, 0.01483837, -0.06013978, 0.04622632],
                            #              [-0.06058618, 0.03173358, -0.99765841, 0.19910626],
                            #              [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)

                            coordinates["base"] = A
                            print("Base_Coordinates A: ", A)
                            #print("Base_Coordinates AA: ", AA)
                            if coordinates["base"] is not None:
                                #print(coordinates)
                                switch.value = 1
                                #print("CONTINUE SPINNING...")
                        # if all(coordinates.get(key) is not None for key in ["base", "small", "medium", "large"]):
                        #     break

                current_time = time.time()
                if current_time - last_print_time >= 2:
                    print(coordinates)
                    last_print_time = current_time
                    print("CONTINUE SPINNING...")

                # cv2.imshow('Camera Feed', frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #    break
                cap.release()  # BUG. without this line RTSP error due to 2 cap object creation. UV. 15.05.2024
            #MODIUFICATION LARGE and BASe
            if all(coordinates.get(key) is not None for key in ["base", "large"]):
                break
            #if all(coordinates.get(key) is not None for key in ["base", "small", "medium", "large"]):
            #    break
        cap.release()
        cv2.destroyAllWindows()
        print("EOP PROCESS STOPPED...")

    return 0


def go_home():
    """
    Abstract: The go_home function is designed to move a robotic arm to its home position. It first establishes a
    TCP connection with the robotic arm. Then, it calls the move_to_home_position function to move the arm to its home
    position. The function provides real-time feedback and stops the process when the arm reaches its home position

    :return:
    """
    #print("GO-HOME PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
    import utilities

    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        move_to_home_position(base)
    print("GO-HOME PROCESS STOPPED..")
    return


def align(base, base_cyclic, pose, x, y):
    """
    Abstract: Moves arm to a specific Cartesian pose. Sets up an action to reach the target pose, which is defined
    by the input parameters x, y, and the pose of the arm. Executes the action and waits for it to finish or for a
    timeout to occur. Provides real-time feedback and stops the process when the arm reaches the target pose or
    a timeout occurs.

    :param base: This is the base of the robotic arm which the function operates on. It’s used to set up the action,
                 execute the action, and subscribe/unsubscribe to action notifications
    :param pose: This is the current pose of the robotic arm.
    :param x, y: target x and y coordinates for the arm to move to.
    :return:
    """
    #print("ALIGN IS CALLED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = x
    cartesian_pose.y = y
    cartesian_pose.z = pose.z
    cartesian_pose.theta_x = pose.theta_x
    cartesian_pose.theta_y = pose.theta_y
    cartesian_pose.theta_z = pose.theta_z

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("ALIGN IS STOPPED...")
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def dangles(angles1, angles2):
    """
    Abstract: Calculates the smallest angular distances between two sets of angles. Takes two lists of angles as
    input, calculates the smallest angular distance using either the smallest_angular_distance_nolimits or
    smallest_angular_distance_limits function. Returns a list of the smallest angular distances.

    :param angles1, angles2: angle lists
    :return:
    """
    #print("DANGLES METHOD IS CALLED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    dangles = np.empty(7)
    dangles[0] = smallest_angular_distance_nolimits(angles1[0], angles2[0])
    dangles[1] = smallest_angular_distance_limits(angles1[1], angles2[1])
    dangles[2] = smallest_angular_distance_nolimits(angles1[2], angles2[2])
    dangles[3] = smallest_angular_distance_limits(angles1[3], angles2[3])
    dangles[4] = smallest_angular_distance_nolimits(angles1[4], angles2[4])
    dangles[5] = smallest_angular_distance_limits(angles1[5], angles2[5])
    dangles[6] = smallest_angular_distance_nolimits(angles1[6], angles2[6])
    return dangles


def euler_to_rotation_matrix(z, y, x):
    """
    Abstract: Converts Euler angles to a rotation matrix. It takes (z, y, x) as input, converts them from degrees
    to radians, and then calculates the rotation matrices Rz, Ry, and Rx for each of the angles. Provides real-time
    feedback and stops the process when the rotation matrices are calculated.

    :param z, y, x: Angles in degrees. Converts these angles to radians and uses them to calculate the rotation matrices.
    :return:
    """
    #print("EULAR TO ROTATION MATRIX METHOD IS CALLED..")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Convert angles from degrees to radians
    z = np.radians(z)
    y = np.radians(y)
    x = np.radians(x)

    Rz = np.array([
        [np.cos(z), -np.sin(z), 0],
        [np.sin(z), np.cos(z), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(y), 0, np.sin(y)],
        [0, 1, 0],
        [-np.sin(y), 0, np.cos(y)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(x), -np.sin(x)],
        [0, np.sin(x), np.cos(x)]
    ])
    print("EULAR TO ROTATION MATRIX METHOD IS STOPPED..")


def rotation_to_nearest_perpendicular(A, B):
    """
    Abstract: Calculates the minimal rotation needed to align a 2D rotation matrix with the nearest principal
    direction (0, 90, 180, or 270 degrees). Computes the relative orientation from one rotation matrix to another, then
    extracts the 2D rotation component. Finds the angles to the nearest X or Y axis alignment and determines the closest
    principal direction. Chooses the smaller of the two angles for the minimal rotation. Provides real-time feedback and
    returns the minimal rotation in degrees.

    :param A, B: Rotation matrices
    :return:
    """
    print("ROTATION TO NEAREST PERPENDICULAR PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Compute the relative orientation C from A to B
    C = np.dot(np.linalg.inv(A), B)

    # Extract the 2D rotation component from C
    # Considering only the X and Y components for simplicity
    rotation_matrix_2d = C[:2, :2]

    # Find the angle to the nearest X or Y axis alignment
    # This is simplified; in practice, you'd need to compute this based on the matrix components
    angles_to_x_axis = np.arctan2(rotation_matrix_2d[1, 0], rotation_matrix_2d[0, 0])
    angles_to_y_axis = np.arctan2(rotation_matrix_2d[0, 1], rotation_matrix_2d[1, 1])

    # Determine the closest principal direction (0, 90, 180, 270 degrees)
    principal_angles = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
    closest_angle_to_x = min(principal_angles, key=lambda x: abs(x - angles_to_x_axis) % np.pi)
    closest_angle_to_y = min(principal_angles, key=lambda x: abs(x - angles_to_y_axis) % np.pi)

    # Choose the smaller of the two angles for the minimal rotation
    min_rotation = min(closest_angle_to_x, closest_angle_to_y, key=abs)
    print("ROTATION TO NEAREST PERPENDICULAR PROCESS STOPPED...")
    return np.degrees(min_rotation)


def section_orientation(x, y):
    """
    Abstract: Calculates the orientation of a section based on the input coordinates (x, y). Calculates the angle
    from the positive x-axis to the point (x, y), converts this angle to degrees, and adjusts the range to [0, 360].
    Depending on the value of the angle, returns a different 3x3 array representing the orientation of the section.

    :param x, y: coordinates of the point that the orientation of the section is calculated
    :return:
    """
    #print("SECTION ORIENTATION METHOD IS CALLED..")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Calculate the angle in radians from the positive x-axis
    angle_rad = np.arctan2(y, x)

    # Convert the angle to degrees
    angle_deg = np.degrees(angle_rad)

    # Adjust the angle range from [0, 360]
    if angle_deg < 0:
        angle_deg += 360

    # Determine and return the circle section based on the angle
    if 45 <= angle_deg < 135:
        return np.array([[-1, 0, 0],
                         [0, 1, 0],
                         [0, 0, -1]])
    elif 135 <= angle_deg < 225:
        return np.array([[0, -1, 0],
                         [-1, 0, 0],
                         [0, 0, -1]])
    elif 225 <= angle_deg < 315:
        return np.array([[1, 0, 0],
                         [0, -1, 0],
                         [0, 0, -1]])
    else:  # Covers both 315 to 360 and 0 to 45 degrees
        return np.array([[0, 1, 0],
                         [1, 0, 0],
                         [0, 0, -1]])




def send_joint_speeds(base, speeds, t):
    """
    Abstract: Controls the joint speeds of DOF7 actuators. Takes the base of the robotic arm, a list of speeds, and
    a duration as input. Sets up the joint speeds for each actuator and sends the joint speeds command to the base.
    Waits for the specified duration before stopping the robot. Provides real-time feedback and returns True when the
    process is stopped.

    :param base: Base of the robotic arm which the function operates on.
    :param speeds: List of speeds for the actuators.
    :param t: Duration for which the function waits after sending the joint speeds command
    :return:
    """
    #print("SEND_JOINT_SPEEDS PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
    # The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1
        print("Sending the joint speeds for 20 seconds...")
        base.SendJointSpeedsCommand(joint_speeds)
        time.sleep(t)

    print("Stopping the robot")
    #print("SEND_JOINT_SPEEDS PROCESS STOPPED..")
    base.Stop()

    return True


def send_joint_speeds1(base, speeds, t):
    """
    Abstract: Controls the joint speeds of DOF7 actuators. Takes the base of the robotic arm, a list of speeds, and
    a duration as input. Sets up the joint speeds for each actuator and sends the joint speeds command to the base.
    Waits for the specified duration before stopping the robot. Provides real-time feedback and returns True when the
    process is stopped.

    :param base: Base of the robotic arm which the function operates on.
    :param speeds: List of speeds for the actuators.
    :param t: Duration for which the function waits after sending the joint speeds command
    :return:
    """
    #print("SEND-JOINT-SPEEDS1 PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")

    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
    # The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1
        print("Sending the joint speeds for 20 seconds...")
        base.SendJointSpeedsCommand(joint_speeds)
        time.sleep(t)

    print("Stopping the robot")
    base.Stop()
    #print("SEND-JOINT-SPEEDS1 PROCESS STOPPED...")
    return True


def calibrate(base, base_cyclic, P):
    """
    Abstract: Moves arm to a specific Cartesian pose defined by a transformation matrix P. Extracts the position and
    orientation from P, sets up an action to reach the target pose, and then executes the action. Waits for the action
    to finish or for a timeout to occur. Provides real-time feedback and returns a boolean indicating whether the
    action was completed successfully.

    :param base: Base of the robotic arm which the function operates on.
    :param P: Transformation matrix that defines the target pose for the arm to move to.
    :return:
    """
    #print("CALIBRATE PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    # terms = [P, T(0,-0.017,0)] 
    # P = np.linalg.multi_dot(terms)
    theta = rot2eul_zyx(P[:3, :3])

    feedback = base_cyclic.RefreshFeedback()
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = P[0, 3]
    cartesian_pose.y = P[1, 3]
    cartesian_pose.z = P[2, 3]
    cartesian_pose.theta_x = theta[0]
    cartesian_pose.theta_y = theta[1]
    cartesian_pose.theta_z = theta[2]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("CALIBRATE PROCESS IS STOPPED..")
        #print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def rot2eul_zyx(R):
    """
    Abstract: Converts a 3x3 rotation matrix to Euler angles. Checks if the input is a valid rotation matrix. Then,
    extracts the three Euler angles (x, y, z) from the rotation matrix. If the rotation matrix is not singular,
    calculates the Euler angles using the arctan2 function. If the rotation matrix is singular, calculates the Euler
    angles in a different way and sets z to 0. Returns the Euler angles in degrees.

    :param R:  a 3x3 rotation matrix.
    :return:
    """
    #print("ROTATION MATRIX METHOD IS CALLED..")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    # Check if the input is a valid rotation matrix
    assert R.shape == (3, 3), "Input must be a  3x3 rotation matrix"

    # Extract the three Euler angles
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    # Convert to degrees if needed
    #print("ROTATION MATRIX METHOD IS STOPPED...")
    return np.degrees([x, y, z])


def vision(base, size):
    """
    Abstract: Captures video from a specific IP camera, detects ArUco markers in the video frames, and estimates
    their pose relative to the camera. Calculates the average translation vector and rotation matrix for the detected
    markers over a one-second interval. Provides visual feedback by displaying the video frames with the detected
    markers and their axes.

    :param base: The base pose of the robot.
    :param size: The size of the ArUco marker.
    :return: If ArUco markers are detected, the function returns the average pose of the markers and their IDs.
    If no markers are detected or an error occurs, the function returns None.
    """
    #print("VISION PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
    #cap = cv2.VideoCapture("rtspsrc location=rtsp://192.168.1.10/color latency=30 !"
    #                      " rtph264depay ! avdec_h264 ! appsink", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    translation_vectors = []
    rotation_vectors = []
    timestamps = []

    last_print_time = time.time()

    #frame_width = int(cap.get(3))
    #frame_height = int(cap.get(4))
    #frame_size = (frame_width, frame_height)

    #result01 = cv2.VideoWriter('filename01.avi',
    #                         cv2.VideoWriter_fourcc(*'MJPG'),
    #                         10, frame_size)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Cannot read frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:

            subprocess.run(['python', vision_sensor_focus_action_filepath])
            for i, marker_id in enumerate(ids.flatten()):
                # Determine the marker size and category
                if 1 <= marker_id <= 6:
                    size = 0.0535
                    s = 0.06
                elif 7 <= marker_id <= 12:
                    size = 0.04
                    s = 0.046
                elif 13 <= marker_id <= 18:
                    size = 0.027
                    s = 0.03
                else:
                    continue

                aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size, cameraMatrix,
                                                                           distCoeffs)
                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 0.03)
                translation_vectors.append(tvecs[0][0])
                rotation_vectors.append(rvecs[0][0])
                timestamps.append(time.time())

            current_time = time.time()
            if current_time - last_print_time >= 1:
                valid_indices = [i for i, t in enumerate(timestamps) if current_time - t <= 1]
                if valid_indices:
                    avg_tvecs = np.mean([translation_vectors[i] for i in valid_indices], axis=0)
                    avg_rvecs = np.mean([rotation_vectors[i] for i in valid_indices], axis=0)

                    rotation_matrix, _ = cv2.Rodrigues(avg_rvecs)
                    pose = base.GetMeasuredCartesianPose()
                    camera_pose = camera_coor(pose)
                    rot_m = T(-avg_tvecs[0], -avg_tvecs[1], avg_tvecs[2])
                    rot_m[:3, :3] = rotation_matrix
                    terms4 = [camera_pose, rot_m]
                    Aruco_matrix = np.linalg.multi_dot(terms4)
                    A = np.dot(Aruco_matrix, T(0, 0, (s / 2)))

                    print(
                        f"Average Translation Vector - X: {-avg_tvecs[0]:.5f}, Y: {-avg_tvecs[1]:.5f}, Z: {avg_tvecs[2]:.5f} meters")
                    print("Average Rotation Matrix:")
                    print(np.array_str(rotation_matrix, precision=5, suppress_small=True))
                    print("Camera pose Matrix:")
                    print(camera_pose)
                    print("Aruco marker Matrix:")
                    print(Aruco_matrix)
                    print("Pick_up Matrix:")
                    print(A)

                    translation_vectors = [translation_vectors[i] for i in valid_indices]
                    rotation_vectors = [rotation_vectors[i] for i in valid_indices]
                    timestamps = [timestamps[i] for i in valid_indices]

                    last_print_time = current_time

                    return A, ids
       #result01.write(frame)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("VISION PROCESS STOPPED...")
    return None


def base_vision(base, size):
    """
    Abstract: Captures video from a specific IP camera, detects ArUco markers in the video frames, and estimates
    their pose relative to the camera. Looks for the ArUco marker with ID 0, then calculates the average translation
    vector and rotation matrix for the detected marker over a one-second interval. Provides visual feedback by
    displaying the video frames with the detected marker and its axes.

    :param base: The base pose of the robot.
    :param size: The size of the ArUco marker.
    :return: If the ArUco marker with ID 0 is detected, returns the average pose of the marker and its ID. If the marker
    is not detected or an error occurs, the function returns None.
    """
    #print("BASE VISION PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    translation_vectors = []
    rotation_vectors = []
    timestamps = []

    last_print_time = time.time()

    #frame_width = int(cap.get(3))
    #frame_height = int(cap.get(4))
    #frame_size = (frame_width, frame_height)

    #result02 = cv2.VideoWriter('filename02.avi',
    #                         cv2.VideoWriter_fourcc(*'MJPG'),
    #                         10, frame_size)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Cannot read frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            #vision_sensor_focus_action_filepath = r'C:\Users\Admin\Desktop\Pick_Place_Demo\Python Pick and Place Files\focus_350.py'
            subprocess.run(['python', vision_sensor_focus_action_filepath])
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != 0:
                    continue  # Skip any marker that is not ID 0

                aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i + 1], size, cameraMatrix,
                                                                           distCoeffs)
                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs, tvecs, size)
                translation_vectors.append(tvecs[0][0])
                rotation_vectors.append(rvecs[0][0])
                timestamps.append(time.time())

            current_time = time.time()
            if current_time - last_print_time >= 1:
                valid_indices = [i for i, t in enumerate(timestamps) if current_time - t <= 1]
                if valid_indices:
                    avg_tvecs = np.mean([translation_vectors[i] for i in valid_indices], axis=0)
                    avg_rvecs = np.mean([rotation_vectors[i] for i in valid_indices], axis=0)

                    rotation_matrix, _ = cv2.Rodrigues(avg_rvecs)
                    pose = base.GetMeasuredCartesianPose()
                    camera_pose = camera_coor(pose)
                    rot_m = T(-avg_tvecs[0], -avg_tvecs[1], avg_tvecs[2])
                    rot_m[:3, :3] = rotation_matrix
                    terms4 = [camera_pose, rot_m]
                    Aruco_matrix = np.linalg.multi_dot(terms4)
                    A = np.dot(Aruco_matrix, T(0, 0, (size / 2)))

                    print(
                        f"Average Translation Vector - X: {-avg_tvecs[0]:.5f}, Y: {-avg_tvecs[1]:.5f}, Z: {avg_tvecs[2]:.5f} meters")
                    print("Average Rotation Matrix:")
                    print(np.array_str(rotation_matrix, precision=5, suppress_small=True))
                    print("Camera pose Matrix:")
                    print(camera_pose)
                    print("Aruco marker Matrix:")
                    print(Aruco_matrix)
                    print("Pick_up Matrix:")
                    print(A)

                    translation_vectors = [translation_vectors[i] for i in valid_indices]
                    rotation_vectors = [rotation_vectors[i] for i in valid_indices]
                    timestamps = [timestamps[i] for i in valid_indices]

                    last_print_time = current_time

                    return A, ids
        #result02.write(frame)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("BASE VISION PROCESS STOPPED...")
    return None


def move(base, base_cyclic, pose, x, y, z):
    """
    Abstract: Executes a Cartesian action movement. Takes in the current pose of the robot and the desired
    displacement in the x, y, and z directions. Calculates the target pose and executes the action. Waits for the
    action to complete or for a timeout to occur.

    :param base: The base control interface of the robot.
    :param base_cyclic: The cyclic control interface of the robot.
    :param pose: The current pose of the robot.
    :param x: The desired displacement in the x direction.
    :param y: The desired displacement in the y direction.
    :param z: The desired displacement in the z direction.
    :return: Returns True if the movement is completed within the timeout duration, False otherwise.
    """
    #print("MOVE PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")

    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pose.x + x
    cartesian_pose.y = pose.y + y
    cartesian_pose.z = pose.z + z
    cartesian_pose.theta_x = pose.theta_x
    cartesian_pose.theta_y = pose.theta_y
    cartesian_pose.theta_z = pose.theta_z

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("MOVE PROCESS STOPPED..")
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def go_base(t, coordinates, angles, base_coor):
    """
    Abstract: Moves the robot to a specified base position. Takes in the desired time for the movement, the
    coordinates of the base, the angles for the robot's joints at the base, and a dictionary to store the final
    coordinates of the base. Calculates the necessary joint speeds for the robot to reach the base position in
    the desired time. Sends these speeds to the robot and waits for the robot to reach the base position. If the
    robot successfully reaches the base position, the function returns 0. If an error occurs, the function returns 1.

    :param t: The desired time for the movement.
    :param coordinates: The coordinates of the base.
    :param angles: The angles for the robot's joints at the base.
    :param base_coor: A dictionary to store the final coordinates of the base.
    :return: Returns 0 if the robot successfully reaches the base position, 1 if an error occurs.
    """
    #print("GO_BASE PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    #print("go to base")
    if angles["base"] is not None:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

        import utilities

        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(
                args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            success = True
            # Create required services
            base1 = BaseClient(router)
            base_cyclic1 = BaseCyclicClient(router)
            s = 0.027
            m = 0.04
            l = 0.0535
            b = 0.0535
            base_x = coordinates['base'][0, 3]
            base_y = coordinates['base'][1, 3]
            TB = np.array([[0, 1, 0, (base_x)],
                           [1, 0, 0, (base_y)],
                           [0, 0, -1, 0.15],
                           [0, 0, 0, 1]])
            RB = section_orientation(base_x, base_y)
            TB[:3, :3] = RB
            TB = np.dot(TB, T(0, -0.054, 0))
            anglesB = angles["base"]
            time.sleep(0.5)
            angles2, pose2 = get_angles_pose(base1)
            time.sleep(0.5)
            dangles0 = dangles(angles2, anglesB)
            speeds1 = dangles0 / t
            success &= send_joint_speeds(base1, speeds1, t)
            success &= calibrate(base1, base_cyclic1, TB)
            base_coor['base'], ids = base_vision(base1, 0.0535)
            print("I AM HERE AFTER BASE_COOR!!!!!!!!!!!!!!!!!!!!!!!")
            return 0 if success else 1
    print("GO-BASE PROCESS IS STOPPED...")


def drop(t, base_coor, angles, x):
    """
    Abstract: Moves the robot to a specified base position and then lower it by a specified distance. Takes in the
    desired time for the movement, the coordinates of the base, the angles for the robot's joints at the base, and the
    distance to lower the robot. Calculates the necessary joint speeds for the robot to reach the base position in the
    desired time. Sends these speeds to the robot and waits for the robot to reach the base position. After reaching
    the base position, the robot is lowered by the specified distance. If the robot successfully completes the
    movement, the function returns 0. If an error occurs, the function returns 1.

    :param t: The desired time for the movement.
    :param base_coor: The coordinates of the base.
    :param angles: The angles for the robot's joints at the base.
    :param x: The distance to lower the robot.
    :return: Returns 0 if the robot successfully completes the movement, 1 if an error occurs.
    """
    #print("DROP PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    print("go to base")
    if base_coor["base"] is not None:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

        import utilities

        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(
                args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            success = True
            # Create required services
            base1 = BaseClient(router)
            base_cyclic1 = BaseCyclicClient(router)
            angles1, pose1 = get_angles_pose(base1)
            success &= move(base1, base_cyclic1, pose1, 0, 0, 0.1)
            s = 0.027
            m = 0.04
            l = 0.0535
            b = 0.0535
            base_x = base_coor['base'][0, 3]
            base_y = base_coor['base'][1, 3]
            TB = np.array([[0, 1, 0, (base_x)],
                           [1, 0, 0, (base_y)],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RB = section_orientation(base_x, base_y)
            TB[:3, :3] = RB
            anglesB = angles["base"]
            time.sleep(0.2)
            angles2, pose2 = get_angles_pose(base1)
            time.sleep(0.2)
            dangles0 = dangles(angles2, anglesB)
            speeds1 = dangles0 / t
            success &= send_joint_speeds(base1, speeds1, t)
            success &= calibrate(base1, base_cyclic1, TB)
            angles3, pose3 = get_angles_pose(base1)
            success &= move(base1, base_cyclic1, pose3, 0, 0, -(pose3.z - x))
            close_gripper(router, router_real_time, 5)
            print("DROP PROCESS STOPPED...")
            return 0 if success else 1


def pick_up_L(t, coordinates, angles):
    """
    Abstract: Moves the robot to a specified large object, pick it up, and then lift it by a specified distance.
    Takes in the desired time for the movement, the coordinates of the object, and the angles for the robot's
    joints at the object. Calculates the necessary joint speeds for the robot to reach the object in then desired
    time. Then sends these speeds to the robot and waits for the robot to reach the object. After reaching the
    object, the robot picks it up and lifts it by the specified distance. If the robot successfully completes
    the movement, the function returns 0. If an error occurs, the function returns 1.

    :param t: The desired time for the movement.
    :param coordinates: The coordinates of the large object.
    :param angles: The angles for the robot's joints at the  object.
    :return: Returns 0 if the robot successfully completes the movement, 1 if an error occurs.
    """
    #print("PICK-UP LARGE PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    #print("pick up large")
    if angles["large"] is not None:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

        import utilities

        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(
                args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            success = True
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            s = 0.027
            m = 0.04
            l = 0.0535
            b = 0.0535
            large_x = coordinates['large'][0, 3]
            large_y = coordinates['large'][1, 3]
            TL = np.array([[0, 1, 0, large_x],
                           [1, 0, 0, large_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RL = section_orientation(large_x, large_y)
            TL[:3, :3] = RL
            TL = np.dot(TL, T(0, -0.054, 0))
            anglesL = angles["large"]
            time.sleep(0.2)
            angles1, pose1 = get_angles_pose(base)
            time.sleep(0.2)
            dangles0 = dangles(angles1, anglesL)
            speeds1 = dangles0 / t
            success &= send_joint_speeds(base, speeds1, t)
            success &= calibrate(base, base_cyclic, TL)
            latest_matrix, ids = vision(base, l)
            print(f"align: {latest_matrix}")
            angles2, pose2 = get_angles_pose(base)
            success &= align(base, base_cyclic, pose2, latest_matrix[0, 3], latest_matrix[1, 3])
            angles3, pose3 = get_angles_pose(base)
            success &= move(base, base_cyclic, pose3, 0, 0, -(pose3.z - 0.04))  # change x y z
            close_gripper(router, router_real_time, 0.057) #0.056
            print("PICK-UP LARGE PROCESS STOPPED...")
            return 0 if success else 1


def pick_up_M(t, coordinates, angles):
    """
    Abstract: Moves the robot to a specified medium object, pick it up, and then lift it by a specified distance.
    Takes in the desired time for the movement, the coordinates of the object, and the angles for the robot's
    joints at the object. Calculates the necessary joint speeds for the robot to reach the object in then desired
    time. Then sends these speeds to the robot and waits for the robot to reach the object. After reaching the
    object, the robot picks it up and lifts it by the specified distance. If the robot successfully completes
    the movement, the function returns 0. If an error occurs, the function returns 1.

    :param t: The desired time for the movement.
    :param coordinates: The coordinates of the medium  object.
    :param angles: The angles for the robot's joints at the  object.
    :return: Returns 0 if the robot successfully completes the movement, 1 if an error occurs.
    """
    #print("PICK-UP MED PROCESS STARTED...")
    #print("pick up medium")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")

    if angles["medium"] is not None:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

        import utilities

        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(
                args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            success = True
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            time.sleep(0.2)
            angles1, pose1 = get_angles_pose(base)
            time.sleep(0.2)
            success &= move(base, base_cyclic, pose1, 0, 0, 0.1)
            s = 0.027
            m = 0.04
            l = 0.0535
            b = 0.0535
            medium_x = coordinates['medium'][0, 3]
            medium_y = coordinates['medium'][1, 3]
            TM = np.array([[0, 1, 0, medium_x],
                           [1, 0, 0, medium_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RM = section_orientation(medium_x, medium_y)
            TM[:3, :3] = RM
            TM = np.dot(TM, T(0, -0.054, 0))
            anglesM = angles["medium"]
            angles2, pose2 = get_angles_pose(base)
            dangles0 = dangles(angles2, anglesM)
            speeds1 = dangles0 / t
            success &= send_joint_speeds(base, speeds1, t)
            success &= calibrate(base, base_cyclic, TM)
            latest_matrix, ids = vision(base, m)
            print(f"align: {latest_matrix}")
            angles3, pose3 = get_angles_pose(base)
            success &= align(base, base_cyclic, pose3, latest_matrix[0, 3], latest_matrix[1, 3])
            angles4, pose4 = get_angles_pose(base)
            success &= move(base, base_cyclic, pose4, 0, 0, -(pose4.z - 0.04))  # change x y z
            close_gripper(router, router_real_time, 0.043)
            print("PICK-UP MED PROCESS STOPPED...")
            return 0 if success else 1


def pick_up_S(t, coordinates, angles):
    """
    Abstract: Moves the robot to a specified small object, pick it up, and then lift it by a specified distance.
    Takes in the desired time for the movement, the coordinates of the object, and the angles for the robot's
    joints at the object. Calculates the necessary joint speeds for the robot to reach the object in then desired
    time. Then sends these speeds to the robot and waits for the robot to reach the object. After reaching the
    object, the robot picks it up and lifts it by the specified distance. If the robot successfully completes
    the movement, the function returns 0. If an error occurs, the function returns 1.

    :param t: The desired time for the movement.
    :param coordinates: The coordinates of the small object.
    :param angles: The angles for the robot's joints at the  object.
    :return: Returns 0 if the robot successfully completes the movement, 1 if an error occurs.
    """
    #print("PICK-UP SMALL PROCESS STARTED...")
    #print("pick up small")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")

    if angles["small"] is not None:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

        import utilities

        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(
                args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            success = True
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            time.sleep(0.2)
            angles1, pose1 = get_angles_pose(base)
            time.sleep(0.2)
            success &= move(base, base_cyclic, pose1, 0, 0, 0.1)
            s = 0.027
            m = 0.04
            l = 0.0535
            b = 0.0535
            small_x = coordinates['small'][0, 3]
            small_y = coordinates['small'][1, 3]
            TS = np.array([[0, 1, 0, small_x],
                           [1, 0, 0, small_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RS = section_orientation(small_x, small_y)
            TS[:3, :3] = RS
            TS = np.dot(TS, T(0, -0.054, 0))
            anglesS = angles["small"]
            angles2, pose2 = get_angles_pose(base)
            dangles0 = dangles(angles2, anglesS)
            speeds1 = dangles0 / t
            success &= send_joint_speeds(base, speeds1, t)
            success &= calibrate(base, base_cyclic, TS)
            latest_matrix, ids = vision(base, m)
            print(f"align: {latest_matrix}")
            angles3, pose3 = get_angles_pose(base)
            success &= align(base, base_cyclic, pose3, latest_matrix[0, 3], latest_matrix[1, 3])
            angles4, pose4 = get_angles_pose(base)
            success &= move(base, base_cyclic, pose4, 0, 0, -(pose4.z - 0.035))  # change x y z
            close_gripper(router, router_real_time, 0.0285)
            print("PICK-UP SMALL PROCESS STOPPED..")
            return 0 if success else 1


def disengage(base, base_cyclic, pos_z):
    """
    Abstract: Moves the robot to a specified position along the z-axis. Takes in the base control interface of the
    robot, the cyclic control interface of the robot, and the desired position along the z-axis. Calculates the target
    pose based on the current pose of the robot and the desired position. Executes the action and waits for the robot
    to reach the target pose or for a timeout to occur. If the robot successfully reaches the target pose, the function
    returns True. If a timeout occurs, the function returns False.

    :param base: The base control interface of the robot.
    :param base_cyclic: The cyclic control interface of the robot.
    :param pos_z: The desired position along the z-axis.
    :return: Returns True if the robot successfully reaches the target pose, False if a timeout occurs.
    """
    action = Base_pb2.Action()
    feedback = base_cyclic.RefreshFeedback()
    print(feedback)
    ##pose = base.GetMeasuredCartesianPose()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x  # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y  # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + pos_z  # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x  # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y  # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z  # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    time.sleep(0)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Pickup location reached\n")
    else:
        print("Timeout on action notification wait\n")
    return finished


def gripper(width):
    """
    Abstract: Controls the gripper of a robot. Takes in the desired width for the gripper. Sends a command to the
    robot to close the gripper to the specified width.

    :param width: The desired width for the gripper.
    :return: None.
    """
    #print("GRIPPER PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

    import utilities

    args = utilities.parseConnectionArguments()

    with utilities.DeviceConnection.createTcpConnection(args) as router, utilities.DeviceConnection.createUdpConnection(
            args) as router_real_time:
        close_gripper(router, router_real_time, width)
    print("GRIPPER PROCESS STOPPED...")
    return


def calc(coordinates, angles):
    """
    Abstract: Calculates the angles for the robot's joints at specified coordinates. Takes in the coordinates of
    different sections and a dictionary to store the calculated angles. Calculates the angles for each section if they
    haven't been calculated yet. The calculation is based on the orientation and position of each section.
    Continues to calculate the angles until all necessary angles are calculated.

    :param coordinates: The coordinates of different sections.
    :param angles: A dictionary to store the calculated angles.
    :return: Returns 0 if all necessary angles are calculated, 1 if an error occurs.
    """
    #print("CALC PROCESS STARTED...")
    current_stack = traceback.extract_stack()
    print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
    while True:
        # Process each coordinate section if it hasn't been processed yet
        if "base" in coordinates and coordinates["base"] is not None and angles.get("base") is None:
            base_x = coordinates["base"][0, 3]
            base_y = coordinates["base"][1, 3]
            TB = np.array([[0, 1, 0, base_x],
                           [1, 0, 0, base_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RB = section_orientation(base_x, base_y)
            TB[:3, :3] = RB
            q1 = angle_guess()
            converged, anglesB, i = trust_constr(q1, TB, 200)
            angles["base"] = anglesB  # Assume anglesB is calculated from your logic

        if "large" in coordinates and coordinates["large"] is not None and angles.get("large") is None:
            large_x = coordinates["large"][0, 3]
            large_y = coordinates["large"][1, 3]
            TL = np.array([[0, 1, 0, large_x],
                           [1, 0, 0, large_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RL = section_orientation(large_x, large_y)
            TL[:3, :3] = RL
            q2 = angle_guess()
            converged, anglesL, i = trust_constr(q2, TL, 200)
            angles["large"] = anglesL

        if "medium" in coordinates and coordinates["medium"] is not None and angles.get("medium") is None:
            medium_x = coordinates["medium"][0, 3]
            medium_y = coordinates["medium"][1, 3]
            TM = np.array([[0, 1, 0, medium_x],
                           [1, 0, 0, medium_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RM = section_orientation(medium_x, medium_y)
            TM[:3, :3] = RM
            q3 = angle_guess()
            converged, anglesM, i = trust_constr(q3, TM, 200)
            angles["medium"] = anglesM

        if "small" in coordinates and coordinates["small"] is not None and angles.get("small") is None:
            small_x = coordinates["small"][0, 3]
            small_y = coordinates["small"][1, 3]
            TS = np.array([[0, 1, 0, small_x],
                           [1, 0, 0, small_y],
                           [0, 0, -1, 0.25],
                           [0, 0, 0, 1]])
            RS = section_orientation(small_x, small_y)
            TS[:3, :3] = RS
            q4 = angle_guess()
            converged, anglesS, i = trust_constr(q4, TS, 200)
            angles["small"] = anglesS

        # MODIFICATION TO JUST BASE AND LARGE CUBE
        # Check if all necessary angles are calculated
        if all(angles.get(key) is not None for key in ['base', 'large']):
            break
        # Check if all necessary angles are calculated
        #if all(angles.get(key) is not None for key in ['base', 'large', 'medium', 'small']):
        #    break
        time.sleep(1)
    print("CALC PROCESS STOPPED...")
    return 0


def main():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
    args = utilities.parseConnectionArguments()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        device_manager = DeviceManagerClient(router)
        vision_config = VisionConfigClient(router)

        current_stack = traceback.extract_stack()
        print(f"{current_stack[-1].name} was called by {current_stack[-2].name}")
        print(datetime.datetime.now())

        vision_device_id = intrinsics.example_vision_get_device_id(device_manager) #9
        intrinsics.example_routed_vision_set_intrinsics(vision_config, vision_device_id)
        intrinsics.example_routed_vision_get_intrinsics(vision_config, vision_device_id)


        manager = multiprocessing.Manager()
        global_return_values = manager.list()
        coordinates = manager.dict({
            #"small": None,
            #"medium": None,
            "large": None,
            "base": None
        })
        angles = manager.dict({
            #"small": None,
            #"medium": None,
            "large": None,
            "base": None
        })
        base_coor = manager.dict({
            "base": None
        })
        switch = manager.Value('i', 1)

        # TO DO: Define processes and Speed. Lower values mean faster movement!!! Start with 8+
        t = 7  # JOINT SPEED ADJUSTMENT!!!
        move_to_home_process = multiprocessing.Process(target=move_to_home_position, args=(base,))
        look_position_process = multiprocessing.Process(target=look_position, args=())
        eop_process = multiprocessing.Process(target=EOP, args=(switch, coordinates,))
        find_process = multiprocessing.Process(target=controller_vision_find,
                                               args=(switch, global_return_values, coordinates,))
        spin_process = multiprocessing.Process(target=spin, args=(switch,))
        calc_process = multiprocessing.Process(target=calc, args=(coordinates, angles,))
        pick_large_process = multiprocessing.Process(target=pick_up_L, args=(t, coordinates, angles,))
        pick_medium_process = multiprocessing.Process(target=pick_up_M, args=(t, coordinates, angles,))
        pick_small_process = multiprocessing.Process(target=pick_up_S, args=(t, coordinates, angles,))
        drop_process = multiprocessing.Process(target=drop, args=(t, base_coor, angles, 0.050,))
        drop2_process = multiprocessing.Process(target=drop, args=(t, base_coor, angles, 0.11,))
        drop3_process = multiprocessing.Process(target=drop, args=(t, base_coor, angles, 0.1525,))
        go_base_process = multiprocessing.Process(target=go_base, args=(t, coordinates, angles, base_coor,))
        #disengage_process = multiprocessing.Process(target=disengage, args=(base, base_cyclic, BASE01_POS_Z,))

        open_gripper_process = multiprocessing.Process(target=gripper, args=(5,))
        close_gripper_process = multiprocessing.Process(target=gripper, args=(0,))
        # Start and join necessary processes
        open_gripper_process.start()
        open_gripper_process.join()
        open_gripper_process.terminate()
        # calc_process.start()

        look_position_process.start()
        look_position_process.join()
        look_position_process.terminate()
        eop_process.start()
        find_process.start()
        time.sleep(10)
        calc_process.start()
        spin_process.start()
        print("STARED")

        # Ensure these processes complete their tasks
        calc_process.join()
        print("FOUND")
        if all(coordinates.get(key) is not None for key in ["base", "large"]) and all(
                angles.get(key) is not None for key in ["base", "large"]):
            find_process.terminate()
            spin_process.terminate()
            eop_process.terminate()
            calc_process.terminate()
            print("TERMINATED")
            print("GOING BASE")
            go_base_process.start()
            go_base_process.join()
            go_base_process.terminate()
            print("GOING LARGE CUBE")
            pick_large_process.start()
            pick_large_process.join()
            pick_large_process.terminate()
            print("GOING BASE")
            drop_process.start()
            drop_process.join()
            drop_process.terminate()
            disengage(base, base_cyclic, BASE01_POS_Z)
            move_to_home_position(base)
            close_gripper_process.start()
            close_gripper_process.join()
            close_gripper_process.terminate()
            #print("GOING MEDIUM CUBE")
            #pick_medium_process.start()
            #pick_medium_process.join()
            #print("MEDIUM PICK PROCESS TERMINATED")
            #pick_medium_process.terminate()
            #print("MEDIUM PICK PROCESS TERMINATED")
            #drop2_process.start()
            #drop2_process.join()
            #drop2_process.terminate()
            #pick_small_process.start()
            #pick_small_process.join()
            #pick_small_process.terminate()
            #drop3_process.start()
            #drop3_process.join()
            #drop3_process.terminate()


if __name__ == "__main__":
    main()
    #input()
