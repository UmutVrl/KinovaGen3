<h1>Vision Examples for Kinova Gen3 Robotic Arm</h1>

<p><strong>Written by:</strong> Umut Can Vural<br>
<strong>Based on:</strong> GitHub: Kinovarobotics/kortex<br>
<strong>For:</strong> KISS Project at Furtwangen University</p>

<h2>Description</h2>

<p>These are vision module examples for the Kinova Gen3 Robotic Arm. </p>

<h2>Demo Videos</h2>

<h2>Features</h2>

<h2>Requirements</h2>

<ul>
  <li>Python 3.9</li>
  <li>Kinova Kortex 2.6.0</li>
  <li>Gen3 firmware Bundle 2.5.2-r.2</li>
  <li>ultralytics 8.2.91</li>
</ul>

<h2>Dependencies</h2>

<ul>
  (pythonProject) PS C:\Users\Student\PycharmProjects\KinovaGen3_Github_Examples> python -m pip freeze
  <br />certifi==2024.8.30
  <br />charset-normalizer==3.3.2
  <br />colorama==0.4.6
  <br />contourpy==1.3.0
  <br />cycler==0.12.1
  <br />Deprecated==1.2.7
  <br />filelock==3.16.0
  <br />fonttools==4.53.1
  <br />fsspec==2024.9.0
  <br />idna==3.8
  <br />importlib_resources==6.4.5
  <br />Jinja2==3.1.4
  <br />kiwisolver==1.4.7
  <br />kortex-api @ file:///C:/Users/Student/Downloads/kortex_api-2.6.0.post3-py3-none-any.whl#sha256=22863493b89ae0ef5270afd58881842cc1814df75736e8ceb0f0d79b869dbbe9
  <br />MarkupSafe==2.1.5
  <br />matplotlib==3.9.2
  <br />mkl-service==2.4.0
  <br />mkl_fft @ file:///C:/b/abs_f55mv94vyg/croot/mkl_fft_1725370278455/work
  <br />mkl_random @ file:///C:/b/abs_21ydbzdu8d/croot/mkl_random_1725370276095/work
  <br />mpmath==1.3.0
  <br />networkx==3.2.1
  <br />numpy==1.26.4
  <br />opencv-python==4.10.0.84
  <br />packaging==24.1
  <br />pandas==2.2.2
  <br />pillow==10.4.0
  <br />protobuf==3.5.1
  <br />psutil==6.0.0
  <br />py-cpuinfo==9.0.0
  <br />pyparsing==3.1.4
  <br />python-dateutil==2.9.0.post0
  <br />pytz==2024.1
  <br />PyYAML==6.0.2
  <br />requests==2.32.3
  <br />scipy==1.13.1
  <br />seaborn==0.13.2
  <br />six==1.16.0
  <br />sympy==1.13.2
  <br />torch==2.4.1
  <br />torchvision==0.19.1
  <br />tqdm==4.66.5
  <br />typing_extensions==4.12.2
  <br />tzdata==2024.1
  <br />ultralytics==8.2.91
  <br />ultralytics-thop==2.0.6
  <br />urllib3==2.2.2
  <br />wrapt==1.16.0
  <br />zipp==3.20.1
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

<p>Run the scripts using Python:</p>

<pre><code>python 02a_color_camera.py</code></pre>

<p>what scripts do: </p>

02a_color_camera: Demonstrates how to access the color camera stream from a Kinova Gen3 robotic arm using Python and OpenCV. 

![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/02a_screenshot.png)

02b_color_detection:


02c_contour_shape_detection:


02e_Yolov8_object_detection:


<h2>Configuration</h2>

<h2>Contributing</h2>

<p>Contributions are welcome! Please feel free to submit a Pull Request.</p>

<h2>License</h2>

<p>This project is part of the KISS Project at Furtwangen University. Please refer to the project's license for usage terms and conditions. https://www.projekt-kiss.net/</p>
  
