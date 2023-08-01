##########################################################
#    Pick and Place Example for Kinova Gen3 Robot        #
#    written by: U. Vural (based on Kinova GitHub)       #
#                                                        #
#    KISS Project at Hochschule Furtwangen University    #
#                                                        #
##########################################################
#    Python 3.9                                          #
#    Kinova Kortex 2.6.0                                 #
#    Gen3 firmware Bundle 2.5.2-r.2                      #
#                                                        #
##########################################################

# Libraries

import sys
import os
import time
import threading

from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, Session_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20
GRIPPER_POS_01 = 0.00
GRIPPER_POS_02 = 0.50
BASE01_POS_X = 0.08
BASE01_POS_Z = 0.40
BASE01_ANG_X = 90

# Create closure to set an event after an END or an ABORT
def check_for_sequence_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications on a sequence

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e = e):
        event_id = notification.event_identifier
        task_id = notification.task_index
        if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
            print("Sequence task {} completed".format(task_id))
        elif event_id == Base_pb2.SEQUENCE_ABORTED:
            print("Sequence aborted with error {}:{}"\
                .format(\
                    notification.abort_details,\
                    Base_pb2.SubErrorCodes.Name(notification.abort_details)))
            e.set()
        elif event_id == Base_pb2.SEQUENCE_COMPLETED:
            print("Sequence completed.")
            e.set()
    return check


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def go_to_home(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    print("Going to default Home Position ...")
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
    time.sleep(0)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Home position reached\n")
    else:
        print("Timeout on action notification wait\n")
    return finished


def go_to_base1(base, base_cyclic, pos_x, pos_z, ang_x): # cartesian_action
    print("Starting Cartesian action movement to go to pickup location...")
    action = Base_pb2.Action()
    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + pos_x # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y   # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + pos_z # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + ang_x # (degrees)
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

def go_to_base2(base): # angular_action
    print("Starting angular action movement to go to dropout location...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    measured_angles = base.GetMeasuredJointAngles()
    #print(measured_angles.joint_angles[0])

    actuator_count = base.GetActuatorCount()
    ##print(action.reach_joint_angles.joint_angles.joint_angles.add().value)
    angles = []
    for joint_id in range(actuator_count.count):
        angles.append(measured_angles.joint_angles[joint_id].value)
    print(angles)

    angles[0] = angles[0] - 180

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

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
        print("Angular movement completed\n")
    else:
        print("Timeout on action notification wait\n")
    return finished


def go_to_base4(base): #twist_command

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0.03
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 5

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True


def pick_and_place_sequence(base, base_cyclic):
    print("Creating Action for Sequence")

    actuator_count = base.GetActuatorCount().count
    angular_action = go_to_base3(actuator_count)
    cartesian_action = go_to_base1(base_cyclic)

    print("Creating Sequence")
    sequence = Base_pb2.Sequence()
    sequence.name = "Example sequence"

    print("Appending Actions to Sequence")
    task_1 = sequence.tasks.add()
    task_1.group_identifier = 0
    task_1.action.CopyFrom(cartesian_action)

    task_2 = sequence.tasks.add()
    task_2.group_identifier = 1  # sequence elements with same group_id are played at the same time
    task_2.action.CopyFrom(angular_action)

    e = threading.Event()
    notification_handle = base.OnNotificationSequenceInfoTopic(
        check_for_sequence_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Creating sequence on device and executing it")
    handle_sequence = base.CreateSequence(sequence)
    base.PlaySequence(handle_sequence)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished


def gripper_control(base, position):
    print("Opening gripper...")
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = position
    finger.finger_identifier = 1
    while position < 1.0:
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        base.SendGripperCommand(gripper_command)
        time.sleep(1)
        break
    print("Gripper is open\n")


def main():
    # Import the utilities helper module
    import argparse
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Sequence

        go_to_home(base)
        #gripper_control(base, GRIPPER_POS_01)
        #go_to_base1(base, base_cyclic, BASE01_POS_X, -BASE01_POS_Z, BASE01_ANG_X)
        #gripper_control(base, GRIPPER_POS_02)
        #go_to_base1(base, base_cyclic, -BASE01_POS_X, BASE01_POS_Z, -BASE01_ANG_X)
        #go_to_home(base)
        go_to_base2(base)
        #go_to_base4(base, base_cyclic)
        #pick_and_place_sequence(base, base_cyclic)



if __name__ == "__main__":
    main()
