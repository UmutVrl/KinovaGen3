<h1>Vision Examples for Kinova Gen3 Robotic Arm</h1>

<p><strong>Written by:</strong> Umut Can Vural<br>
<strong>Based on:</strong> GitHub: Kinovarobotics/kortex<br>
<strong>For:</strong> KISS Project at Furtwangen University</p>

<h2>Description</h2>

**KINOVA_PICK_AND_PLACE_DEMO:**

<p>KINOVA_PICK_AND_PLACE_DEMO.py implements a pick-and-place operation using a Kinova Gen3 robotic arm. It utilizes computer vision with ArUco markers for object detection and positioning. The code includes functions for moving the arm to the home position, controlling joint movements, calculating camera coordinates, and detecting ArUco markers in the video stream. It uses multithreading to improve performance and includes camera calibration and focus adjustment. The main controller function (controller_vision_find) continuously captures frames from the robot's camera, detects ArUco markers, and updates global variables based on the detected markers. The script is designed to work with specific hardware configurations and includes safety measures and error handling. It's a modified version of a pick and place example, tailored for detecting and manipulating a large cube and a base marker. </p>

**Camera_Calibration:**

<p>Camera_Calibration.py script implements a camera calibration process for a Kinova Gen3 robotic arm. It uses OpenCV to calibrate the camera using a chessboard pattern. The script captures multiple images of a chessboard from different angles, detects chessboard corners in each image, and uses these to calculate the camera matrix and distortion coefficients. These calibration results are crucial for accurate computer vision tasks with the robotic arm, such as object detection and pose estimation. The script includes functionality to save the calibration results to files for later use, and it measures the time taken for the calibration process. It's designed to work with specific hardware configurations and includes error handling and detailed output of calibration data. This calibration is an essential step in setting up the robotic arm for vision-based tasks, ensuring accurate 3D to 2D projections and image undistortion.</p>

<h2>Demo Video</h2>

<h2>Features</h2>

<h2>Requirements</h2>

<ul>
  <li>Python 3.9</li>
  <li>Kinova Kortex 2.6.0</li>
  <li>Gen3 firmware Bundle 2.5.2-r.2</li>
  <li>OpenCV Contrib (opencv-contrib-python 4.5.*) ! See Notes Below !</li>
  <li> NumPy 1.26.4</li>
  <li>Pandas 2.2.1</li>
  <li>Matplotlib 3.8.1</li>
  <li>SciPy 1.12.0</li>
</ul>

<h2>Dependencies</h2>

<ul>
 <pre><code)(venv) PS C:\Users\Student\PycharmProjects\KinovaGen3_GitHub_Examples_Part2> python -m pip freeze                   
 <br />contourpy==1.3.0
 <br />cycler==0.12.1
 <br />Deprecated==1.2.7
 <br />fonttools==4.53.1
 <br />importlib_resources==6.4.5
 <br />kiwisolver==1.4.7
 <br />kortex-api @ file:///C:/Users/Student/PycharmProjects/KinovaGen3_GitHub_Examples_Part2/venv/kortex_api-2.6.0.post3-py3-none-any.whl#sha256=22863493b89ae0ef5270afd58881842cc1814df75736e8ceb0f0d79b869dbbe9
 <br />matplotlib==3.8.1
 <br />numpy==1.26.4
 <br />opencv-contrib-python==4.5.5.64
 <br />packaging==24.1
 <br />pandas==2.2.1
 <br />pillow==10.4.0
 <br />protobuf==3.5.1
 <br />pyparsing==3.1.4
 <br />python-dateutil==2.9.0.post0
 <br />pytz==2024.1
 <br />scipy==1.12.0
 <br />six==1.16.0
 <br />tzdata==2024.1
 <br />wrapt==1.16.0
 <br />zipp==3.20.1
 </code></pre>
</ul>

<h2>Installation</h2>

<ol>
  <li>Ensure you have Python 3.9 installed on your system.</li>
  <li>Ensure that you have installed dependencies. (See Dependencies) </li>
  <li>Install the Kinova Kortex library:
    <pre><code>pip install kortex_api</code></pre>
  </li>
  <li>Clone this repository:
    <pre><code>git clone https://github.com/yourusername/kinova-gen3-pick-and-place.git
cd kinova-gen3-pick-and-place</code></pre>
  </li>
</ol>

<h2>Usage</h2>

<ol>
  <li>1 - Prepare the Cubes and a Base Plate. </li>
   <li>Big Cube Dimensions: (ArUco Numbers 1-6)  </li>
   <li>Medium Cube Dimensions: (ArUco Numbers 7-12)</li>
   <li>Small Cube Dimensions: (ArUco Numbers 13-18) </li>
   <li>Base Plate Dimensions: (ArUco Numbers 0) </li>
  <li>2- Calibration</li>

Screenshot
 
</ol>

<p>Run the scripts using Python:</p>

<pre><code>python KINOVA_PICK_AND_PLACE_DEMO.py [connection_arguments] </code></pre>

<h2>Notes</h2>

<br /> **Calibration**
<br /> Camera calibration is a very important step. Make sure that you get the correct dist and camera matrix values. You need to change these values in your main script.
<br /> Be careful with chessboard dimensions. You need to count the inner column % row numbers. 
<br /> Take as many photos as you can (100+) covering different angles. Focus on taking more photos in the robot operation distance

<br /> **ArUco_Markers + Cubes**
<br /> You can 3d print the cubes.
<br /> You need to enter ArUco Marker dimensions into the KINOVA_PICK_AND_PLACE_DEMO.py script.

<br /> **Dependencies**
<br />Example03_py dependencies are different than other examples. For instance, opencv-python version is 4.5.* See others below 
<br />If opencv-python 4.5.* and opencv_contrib-python 4.5.* installation leads cv2.aruco module problems try this:
https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco

<br /> **Troubleshooting & Warnings**
 <br /> This script is still in the experimental phase. Expect glitches and malfunctions.
 <br /> Ensure objects are not placed too close to each other or the edges of the workspace. Put the objects one by one, after the model iterations are finished.
 <br /> The Current work area is too small. You can enable the spinning or make the robot scan more working space and put distances between cubes. This should help with the malfunctions because of object closeness.
 <br /> Operate Gen3 in a SAFE environment & OPEN Space! 
 <br /> While testing & modifying the code,it would be wise to run Gen3 slower first (see "line t = 7  # JOINT SPEED ADJUSTMENT!!!" in main) 
 <br /> Monitor the movement of the robot and script logs while running.
 

<h2>Contributing</h2>

<p>Contributions are welcome! Please feel free to submit a Pull Request.</p>

<h2>Acknowledgements</h2>

<p>This script is a simplified and modified version of the University of Calgary ENME 501/502 Group's Pick & Place example. Original work can be found at:
https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial </p>

<h2>License</h2>

<p>This project is part of the KISS Project at Furtwangen University. Please refer to the project's license for usage terms and conditions. https://www.projekt-kiss.net/</p>
  


