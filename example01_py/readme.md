# Demo Video
![](https://github.com/UmutVrl/KinovaGen3/blob/main/media/01_pick_and_place.gif)

# Notes
This Python program is designed to control a robotic arm (specifically, the Kinova Gen3 Robotic Arm) to perform a pick-and-place sequence. Here’s a brief overview of what it does:

1- Setup: The program sets up a connection to the robotic arm and prepares for the pick-and-place sequence.

2- Pick-and-Place Sequence: The sequence involves the following steps:
  <br /> -Go to Home: The robotic arm is moved to a default ‘Home’ position.
  <br /> -Open Gripper: The gripper of the robotic arm is fully opened.
  <br /> -Go to Pickup Location: The robotic arm is moved to a specified pickup location using Cartesian action movement.
  <br /> -Close Gripper: The gripper is closed to pick up an object.
  <br />-Go to Home: The robotic arm is moved back to the ‘Home’ position with the object.
  <br />-Go to Dropout Location: The robotic arm is moved to a specified dropout location using angular action movement and twist command movement.
  <br />-Open Gripper: The gripper is opened to drop the object.
  <br />-Go to Home: Finally, the robotic arm is moved back to the ‘Home’ position.

Each movement step waits for completion before proceeding to the next step. If a step is not completed within a specified timeout duration, the program prints a timeout message.

This program is useful in scenarios where you need to automate the process of picking up an object from one location and placing it in another location using a robotic arm. The sequence can be customized by changing the parameters for the pickup and dropout locations, as well as the gripper positions. Please note that the dimensions of the frames should match the main program where these images will be used.

# Dependencies
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