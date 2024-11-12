<h1>Pick-and-place Example II for Kinova Gen3 Robotic Arm</h1>

<p><strong>Written by:</strong> Umut Can Vural<br>
<strong>Based on:</strong> GitHub: Kinovarobotics/kortex & GitHub: CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial<br>
<strong>For:</strong> KISS Project at Furtwangen University</p>

<h2>Description</h2>

**KINOVA_PICK_AND_PLACE_DEMO:**

<p>KINOVA_PICK_AND_PLACE_DEMO.py implements a pick-and-place operation using a Kinova Gen3 robotic arm. It utilizes computer vision with ArUco markers for object detection and positioning. The code includes functions for moving the arm to the home position, controlling joint movements, calculating camera coordinates, and detecting ArUco markers in the video stream. It uses multithreading to improve performance and includes camera calibration and focus adjustment. The main controller function (controller_vision_find) continuously captures frames from the robot's camera, detects ArUco markers, and updates global variables based on the detected markers. The script is designed to work with specific hardware configurations and includes safety measures and error handling. It's a modified version of a pick and place example, tailored for detecting and manipulating a large cube and a base marker. </p>

**Camera_Calibration:**

<p>Camera_Calibration.py script implements a camera calibration process for a Kinova Gen3 robotic arm. It uses OpenCV to calibrate the camera using a chessboard pattern. The script captures multiple images of a chessboard from different angles, detects chessboard corners in each image, and uses these to calculate the camera matrix and distortion coefficients. These calibration results are crucial for accurate computer vision tasks with the robotic arm, such as object detection and pose estimation. The script includes functionality to save the calibration results to files for later use, and it measures the time taken for the calibration process. It's designed to work with specific hardware configurations and includes error handling and detailed output of calibration data. This calibration is an essential step in setting up the robotic arm for vision-based tasks, ensuring accurate 3D to 2D projections and image undistortion.</p>

**Camera_Calibration_part2:**

<p>Camera_Calibration_part2.py performs camera calibration and image undistortion using OpenCV. It starts by detecting chessboard corners in a series of calibration images, using these to compute the camera's intrinsic parameters (camera matrix and distortion coefficients). The calibration results are then saved for future use. The script demonstrates two methods of image undistortion: direct undistortion using cv2.undistort() and remapping using cv2.initUndistortRectifyMap() and cv2.remap(). It also calculates the reprojection error to assess the accuracy of the calibration. Throughout the process, the script measures and reports execution time. This calibration allows for the correction of lens distortions in images captured by the camera, which is crucial for accurate computer vision applications, particularly in robotics contexts like the Kinova Gen3 robotic arm mentioned in the script's header.</p>

**Screenshot_Taker_timer:**

<p>Screenshot_Taker_timer.py captures calibration images from an RTSP video stream, specifically designed for camera calibration purposes. It connects to a camera feed, disables autofocus, and continuously reads frames while displaying the video output. Every three seconds, it saves the current frame as a JPEG image in a specified directory, overlaying a "Scan Saved" message on the video feed to indicate successful captures. The script allows users to collect a diverse set of images by repositioning the calibration pattern between screenshots, which is essential for accurate camera calibration. The captured images will be used to determine the camera's intrinsic parameters and distortion coefficients, facilitating improved computer vision applications in robotics. The process continues until the user presses the ESC key to exit.</p>

<h2>Demo Video</h2>

![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/pickplace_demo.gif)

<h2>Features</h2>

<h2>Requirements</h2>

<ul>
  <li>Python 3.9</li>
  <li>Kinova Kortex 2.6.0</li>
  <li>Gen3 firmware Bundle 2.5.2-r.2</li>
  <li>OpenCV Contrib (opencv-contrib-python 4.5.*) ! See Notes Below !</li>
  <li>NumPy 1.26.4</li>
  <li>Pandas 2.2.1</li>
  <li>Matplotlib 3.8.1</li>
  <li>SciPy 1.12.0</li>
</ul>

<h2>Dependencies</h2>

<ul>
 <pre><code><br />(pythonProject) PS C:\Users\Student\PycharmProjects\KinovaGen3_Github_Examples> python -m pip freeze               
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
  <li>Ensure that you have installed dependencies. (See Dependencies) 
  <pre><code>
python -m pip install matplotlib==3.8.1
python -m pip install pandas==2.2.1
python -m pip install scipy==1.12.0
python -m pip install opencv-contrib-python==4.5.5.64
    </code></pre>
  </li>
  <li>Install the Kinova Kortex library:
    <pre><code>
python -m pip install .\kortex_api-2.6.0.post3-py3-none-any.whl
    </code></pre>
  </li>
  <li>Clone this repository:
    <pre><code>git clone https://github.com/yourusername/kinova-gen3-pick-and-place.git
cd kinova-gen3-pick-and-place</code></pre>
  </li>
</ol>

<h2>Usage</h2>

<ol>
  <li> Prepare the Cubes and a Base Plate. Prepare ArUco markers. The dimensions are smaller than cube dimensions and you need to modify them in the script. (see /resources)
  <br />Big Cube Dimensions: 6.0 x 6.0 x 6.0 (cm) (ArUco Numbers 1-6)
  <br />Medium Cube Dimensions: 4.5 x 4.5 x 4.5 (cm) (ArUco Numbers 7-12)
  <br />Small Cube Dimensions: 3.0 x 3.0 x 3.0 (cm) (ArUco Numbers 13-18)
  <br />

![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/ArUco_cubes.jpg)
    
  <li>Calibration
  <br /> Use Camera_Calibration.py and Camera_Calibration_part2.py (see /resources)
  <br /></li>
  
![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/cam_calibration.png)
  
 
 <li>Run the scripts using Python:
   <pre><code>python KINOVA_PICK_AND_PLACE_DEMO.py </code></pre>
 </li>

</ol>



<h2>Notes</h2>

<br /> **Calibration**
<br /> Camera calibration is a crucial step. Make sure that you get the correct distortion and camera matrix values. You need to change these values in your main script.
<br /> Be careful with chessboard dimensions. You need to count the inner column % row numbers. 
<br /> Take as many photos as you can (100+) covering different angles. Focus on taking more photos in the robot operation distance
[calibration correctness](https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056)
[camera calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

<br /> **ArUco_Markers + Cubes**
<br /> You can 3d print the cubes.
<br /> You need to enter ArUco Marker dimensions into the KINOVA_PICK_AND_PLACE_DEMO.py script.
[aruco opencv doc](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) (opencv 4.5.5)


<br /> **Dependencies**
<br />Example03_py dependencies are different than other examples. For instance, opencv-python version is 4.5.* See others below 
<br />If opencv-python 4.5.* and opencv_contrib-python 4.5.* installation leads cv2.aruco module problems try this:
[no module named aruco](https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco)

<br /> **Troubleshooting & Warnings**
 
 <br /> This script is still in the experimental phase. Expect glitches and malfunctions.
 <br /> Ensure objects are not placed too close to each other or the edges of the workspace. Put the objects one by one, after the model iterations are finished.
 <br /> The Current work area is too small. You can enable the spinning or make the robot scan more space and therefore put cubes away from each other. This should help with the malfunctions because of object closeness.
 <br /> While testing & modifying the code,it would be wise to run Gen3 slower first (see "line t = 7  # JOINT SPEED ADJUSTMENT!!!" in main) 
 <br /> Monitor the movement of the robot and script logs while running.
 <br /> Operate Gen3 in a SAFE environment & OPEN Space! 

<h2>Contributing</h2>

<p>Contributions are welcome! Please feel free to submit a Pull Request.</p>

<h2>Acknowledgements</h2>

<p>This script is a simplified and modified version of the University of Calgary ENME 501/502 Group's Pick & Place example. Original work can be found at:
https://github.com/CassGut/Kinova-Gen-3-Robotic-Arm-Pick-and-Place-Controller-Tutorial </p>

<h2>License</h2>

<p>This project is part of the KISS Project at Furtwangen University. Please refer to the project's license for usage terms and conditions. https://www.projekt-kiss.net/</p>
  


