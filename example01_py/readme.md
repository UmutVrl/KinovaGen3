<h1>Pick and Place Example for Kinova Gen3 Robotic Arm</h1>

<p><strong>Written by:</strong> Umutcan Vural<br>
<strong>Based on:</strong> GitHub: Kinovarobotics/kortex<br>
<strong>For:</strong> KISS Project at Furtwangen University</p>

<h2>Description</h2>

<p>This project uses a Kinova Gen3 Robotic Arm to demonstrate a pick-and-place operation. The script controls the arm to perform a sequence of movements, including gripper control, to pick up an object from one location (Location A) and place it in another (Location B). The user manually defined the robot's movement locations inside the code. </p>

<h2>Demo Video</h2>
![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/01_pick_and_place.gif)

<h2>Features</h2>

<ul>
  <li>Home position movement</li>
  <li>Cartesian action movement</li>
  <li>Angular action movement</li>
  <li>Twist command movement</li>
  <li>Gripper control</li>
</ul>

<h2>Requirements</h2>

<ul>
  <li>Python 3.9</li>
  <li>Kinova Kortex 2.6.0</li>
  <li>Gen3 firmware Bundle 2.5.2-r.2</li>
</ul>

<h2>Dependencies</h2>

<ul>
  <br />(pythonProject) PS C:\Users\Student\PycharmProjects\KinovaGen3_Github_Examples> python -m pip freeze
  ...
  <br />kortex-api @ file:///C:/Users/Student/Downloads/kortex_api-2.6.0.post3-py3-none-any.whl#sha256=22863493b89ae0ef5270afd58881842cc1814df75736e8ceb0f0d79b869dbbe9
  ...

<h2>Installation</h2>

<ol>
  <li>Ensure you have Python 3.9 installed on your system.</li>
  <li>Ensure that you have installed dependencies.(eg. time, threading) </li>
  <li>Install the Kinova Kortex library:
    <pre><code>pip install kortex_api</code></pre>
  </li>
  <li>Clone this repository:
    <pre><code>git clone https://github.com/yourusername/kinova-gen3-pick-and-place.git
cd kinova-gen3-pick-and-place</code></pre>
  </li>
</ol>

<h2>Usage</h2>

<p>Run the script using Python:</p>

<pre><code>python pick_and_place.py</code></pre>

<p>Here’s a brief overview of what the script does: </p>

1- Setup: The program sets up a connection to the robotic arm and prepares for the pick-and-place sequence.

2- Pick-and-Place Sequence: The sequence involves the following steps:
  <br /> -Go to Home: The robotic arm is moved to a default ‘Home’ position.
  <br /> -Open Gripper: The gripper of the robotic arm is fully opened.
  <br /> -Go to Pickup Location: The robotic arm is moved to a specified pickup location using Cartesian action movement.
  <br /> -Close Gripper: The gripper is closed to pick up an object
  <br /> -Go to Home: The robotic arm is moved back to the ‘Home’ position with the object.
  <br /> -Go to Dropout Location: The robotic arm is moved to a specified dropout location using angular action movement and twist command movement.
  <br /> -Open Gripper: The gripper is opened to drop the object.
  <br /> -Go to Home: Finally, the robotic arm is moved back to the ‘Home’ position.

Each movement step waits for completion before proceeding to the next step. If a step is not completed within a specified timeout duration, the program prints a timeout message.

This program is useful in scenarios where you need to automate the process of picking up an object from one location and placing it in another location using a robotic arm. The sequence can be customized by changing the parameters for the pickup and dropout locations, as well as the gripper positions. Please note that the dimensions of the frames should match the main program where these images will be used.

<h2>Configuration</h2>

<p>You can adjust the following parameters in the script:</p>

<ul>
  <li><code>TIMEOUT_DURATION</code>: Maximum allowed waiting time during actions (in seconds)</li>
  <li><code>GRIPPER_POS_01</code>: Gripper position for fully open (0.00)</li>
  <li><code>GRIPPER_POS_02</code>: Gripper position for half closed (0.50)</li>
  <li><code>BASE01_POS_X</code>: X-axis position for pickup location (0.08 meters)</li>
  <li><code>BASE01_POS_Z</code>: Z-axis position for pickup location (0.40 meters)</li>
  <li><code>BASE01_ANG_X</code>: X-axis angle for pickup location (90 degrees)</li>
  <li><code>A0_TURN_DEGREE</code>: Actuator ID0 turn degree (80 degrees)</li>
  <li><code>A5_TURN_DEGREE</code>: Actuator ID5 turn degree (90 degrees)</li>
</ul>

<h2>Contributing</h2>

<p>Contributions are welcome! Please feel free to submit a Pull Request.</p>

<h2>License</h2>

<p>This project is part of the KISS Project at Furtwangen University. Please refer to the project's license for usage terms and conditions. https://www.projekt-kiss.net/</p>



