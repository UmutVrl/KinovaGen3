# Demo Video
![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/01_pick_and_place.gif)


This Python program is designed to control a robotic arm (specifically, the Kinova Gen3 Robotic Arm) to perform a pick-and-place sequence. Here’s a brief overview of what it does:

1- Setup: The program sets up a connection to the robotic arm and prepares for the pick-and-place sequence.

2- Pick-and-Place Sequence: The sequence involves the following steps:
  -Go to Home: The robotic arm is moved to a default ‘Home’ position.
  -Open Gripper: The gripper of the robotic arm is fully opened.
  -Go to Pickup Location: The robotic arm is moved to a specified pickup location using Cartesian action movement.
  -Close Gripper: The gripper is closed to pick up an object.
  -Go to Home: The robotic arm is moved back to the ‘Home’ position with the object.
  -Go to Dropout Location: The robotic arm is moved to a specified dropout location using angular action movement and twist command movement.
  -Open Gripper: The gripper is opened to drop the object.
  -Go to Home: Finally, the robotic arm is moved back to the ‘Home’ position.

Each movement step waits for completion before proceeding to the next step. If a step is not completed within a specified timeout duration, the program prints a timeout message.

This program is useful in scenarios where you need to automate the process of picking up an object from one location and placing it in another location using a robotic arm. The sequence can be customized by changing the parameters for the pickup and dropout locations, as well as the gripper positions. Please note that the dimensions of the frames should match the main program where these images will be used.
