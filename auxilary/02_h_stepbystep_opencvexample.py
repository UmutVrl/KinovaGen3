
# Based on official librealsense example:
# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
# TODO: Modify this program for the Kinova-robot (08.23)

# Import libraries
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

# creating objects
# The config allows pipeline users to request filters for the pipeline streams and device selection and configuration.
config = rs.config()
# The pipeline class  the camera configuration and streaming, and the vision modules triggering and threading
pipeline = rs.pipeline()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == "RGB Camera":
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    while True:

        # (1) Wait for a coherent pair of frames: depth and color frames are present for assemble
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # (2) Convert images to numpy array representations
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # (3) Convert array to an 8-bit pixel map (for human perception)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # (4) Apply the color and depth frames to the same resolution
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # (5) Save an image and stream the depth and RGB cameras
        cv2.imwrite("test.png", images)
        cv2.namedWindow("Live Stream", cv2.WINDOW_FULLSCREEN)
        cv2.imshow("Live Stream", images)
        cv2.waitKey(1)

        # (6) stop stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
