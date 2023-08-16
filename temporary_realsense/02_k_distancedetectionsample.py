# Based on https://pysource.com/2021/03/11/distance-detection-with-depth-camera-intel-realsense-d435i/ by Sergio Canu

# Import libraries
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

SCREEN_WIDTH = 480
SCREEN_LENGTH = 640
FRAME_RATE = 30
CIRCLE_COLOR = (0, 255, 255)
CIRCLE_THICKNESS = 5
CIRCLE_RADIUS = 0
TEXT_LOCATION = (50, 50)
TEXT_THICKNESS = 2
TEXT_COLOR = (0, 0, 0)
TEXT_SCALE = 2


class RealsenseCamera:
    # Based on official librealsense example:
    # https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
    def __init__(self):
        # creating objects
        # The config allows pipeline users to request filters for the pipeline streams and
        # device selection and configuration.
        config = rs.config()
        # The pipeline class  the camera configuration and streaming, and the vision modules triggering and threading
        self.pipeline = rs.pipeline()

        config.enable_stream(rs.stream.depth, SCREEN_LENGTH, SCREEN_WIDTH, rs.format.z16, FRAME_RATE)
        config.enable_stream(rs.stream.color, SCREEN_LENGTH, SCREEN_WIDTH, rs.format.bgr8, FRAME_RATE)

        self.pipeline.start(config)

    def assemble_frame(self):
        # (1) Wait for a coherent pair of frames: depth and color frames are present for assemble
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # (2) Convert images to numpy array representations
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # (3) if both depth and color frames are valid and not None
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def stream_frame(self):
        # (5) Save an image and stream the depth and RGB cameras
        cv2.imwrite("test.png", self.images)
        cv2.namedWindow("Live Stream", cv2.WINDOW_FULLSCREEN)
        cv2.imshow("Live Stream", self.images)
        cv2.waitKey(1)

    def stop_pipeline(self):
        # Stop streaming
        self.pipeline.stop()


if __name__ == '__main__':
    # Initialize the Camera
    cam = RealsenseCamera()

    while True:
        ret, depth_frame, color_frame = cam.assemble_frame()

        # Show distance for screen center
        center_point = (int(SCREEN_LENGTH/2), int(SCREEN_WIDTH/2))

        cv2.circle(color_frame, center_point, CIRCLE_RADIUS, CIRCLE_COLOR, CIRCLE_THICKNESS)
        distance = depth_frame[center_point[1], center_point[0]]
        print(f"Distance is {distance} mm ")
        cv2.putText(color_frame, "{}mm".format(distance), (TEXT_LOCATION[1], TEXT_LOCATION[0]),
                    cv2.FONT_HERSHEY_PLAIN, TEXT_SCALE, TEXT_COLOR, TEXT_THICKNESS)

        cv2.imshow("Color_Frame", color_frame)
        #cv2.imshow("Depth_Frame", depth_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break






