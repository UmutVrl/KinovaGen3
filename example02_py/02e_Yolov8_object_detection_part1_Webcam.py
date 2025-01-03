##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Multithreading + Yolov8 object detection            #
#    Part1 uses Webcam to test multithreading and        #
#    Yolov8 model                                        #
#                                                        #
#    written by: U. Vural                                #
#    based on: GitHub:Kinovarobotics/kortex              #
#                                                        #
#    for KISS Project at Furtwangen University           #
#                                                        #
##########################################################
#    specs:                                              #
#    Python 3.9                                          #
#    Kinova Kortex 2.6.0                                 #
#    Gen3 firmware Bundle 2.5.2-r.2                      #
##########################################################
#    requirements:                                       #
#    opencv-python                                       #
#    numpy                                               #
#    matplotlib                                          #
#    ultralytics                                         #
#                                                        #
##########################################################
# multi-threading source:
# https://github.com/nrsyed/computer-vision
# https://nrsyed.com/2018/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/


# SUMMARY
# In our instance, real bottleneck is not camera stream is not reading and decoding video frames.
# Threading is not efficient as expected in current load. Simply better HW should be effective.
# Iteration Counts per model:
# noThreading()  # 6 iterations per second / 29 iterations per second [with / without Yolov8]
# threadVideoGet()  # 6 iterations per second / 400 iterations per second [with / without Yolov8]
# threadVideoShow()  # 4 iterations per second / 30 iterations per second [with / without Yolov8]
# threadVideoBoth()  # 4 iterations per second / 40K iterations per second [with / without Yolov8]

# Libraries

import cv2
from ultralytics import YOLO
from threading import Thread
import math
from datetime import datetime


class CountsPerSec:
    """
    https://github.com/nrsyed/computer-vision
    Class that tracks the number of occurrences ("counts") of an
    arbitrary event and returns the frequency in occurrences
    (counts) per second. The caller must increment the count.
    """

    def __init__(self):
        self._start_time = None
        self._num_occurrences = 0

    def start(self):
        self._start_time = datetime.now()
        return self

    def increment(self):
        self._num_occurrences += 1

    def counts_per_sec(self):
        elapsed_time = (datetime.now() - self._start_time).total_seconds()
        return self._num_occurrences / elapsed_time


class VideoGet:
    """
    https://github.com/nrsyed/computer-vision
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoShow:
    """
    https://github.com/nrsyed/computer-vision
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("Video", self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def stop(self):
        self.stopped = True


def putIterationsPerSec(frame, iterations_per_sec):
    """
    https://github.com/nrsyed/computer-vision
    Add iterations per second text to lower-left corner of a frame.
    """

    cv2.putText(frame, "{:.0f} iterations/sec".format(iterations_per_sec),
                (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
    return frame


def noThreading(source=0):
    """
    Yolov8 Object[Cup] detection without multithreading.
    """
    cap = cv2.VideoCapture(source)
    cps = CountsPerSec().start()
    # YOLOv8n is the fastest option
    # See github.com/ultralytics/ultralytics
    model = YOLO("yolov8n.pt")
    # model.info()
    # ClassNames List [0:79]
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
                  "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                  "teddy bear", "hair drier", "toothbrush"
                  ]

    while True:
        grabbed, frame = cap.read()
        # frame = putIterationsPerSec(frame, cps.counts_per_sec())
        if not grabbed or cv2.waitKey(1) == ord("q"):
            break

        frame = putIterationsPerSec(frame, cps.counts_per_sec())
        # cv2.imshow("Video2", frame)
        cps.increment()
        # Object detection using YOLOv8, frame by frame
        # Classes=41 to filter Cup Class
        # Adjust Confidence Interval
        # Stream=True to generate a memory-efficient generator of Results objects.
        results = model(frame, stream=True, conf=0.5, classes=41)
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                # print(x1, y1, x2, y2)

                # Draw the bounding box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Calculate confidence value
                conf = math.ceil((box.conf[0] * 100)) / 100

                # Get the class number and class name
                cls = int(box.cls[0])
                class_name = classNames[cls]

                # Write the class name and confidence value
                label = f"{class_name}{conf}"
                text_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                c2 = x1 + text_size[0], y1 - text_size[1] - 3
                cv2.rectangle(frame, (x1, y1), c2, [255, 0, 0], -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, y1 - 2), 0,
                            1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
            resized_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            cv2.imshow("Video", resized_frame)

    cap.release()
    cv2.destroyAllWindows()


def threadVideoGet(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Main thread serves to Object[Cup] detection using Yolov8.
    """

    video_getter = VideoGet(source).start()
    cps = CountsPerSec().start()
    # YOLOv8n is the fastest option
    # See github.com/ultralytics/ultralytics
    model = YOLO("yolov8n.pt")
    # model.info()
    # ClassNames List [0:79]
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
                  "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                  "teddy bear", "hair drier", "toothbrush"
                  ]

    while True:
        frame = video_getter.frame
        if (cv2.waitKey(1) == ord("q")) or video_getter.stopped:
            video_getter.stop()
            break

        frame = putIterationsPerSec(frame, cps.counts_per_sec())
        # cv2.imshow("Video2", frame)
        cps.increment()
        # Object detection using YOLOv8, frame by frame
        # Classes=41 to filter Cup Class
        # Adjust Confidence Interval
        # Stream=True to generate a memory-efficient generator of Results objects.
        results = model(frame, stream=True, conf=0.3, classes=41)
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                print(x1, y1, x2, y2)

                # Draw the bounding box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Calculate confidence value
                conf = math.ceil((box.conf[0] * 100)) / 100

                # Get the class number and class name
                cls = int(box.cls[0])
                class_name = classNames[cls]

                # Write the class name and confidence value
                label = f"{class_name}{conf}"
                text_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                c2 = x1 + text_size[0], y1 - text_size[1] - 3
                cv2.rectangle(frame, (x1, y1), c2, [255, 0, 0], -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, y1 - 2), 0,
                            1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
            resized_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            cv2.imshow("Video", resized_frame)

    video_getter.stop()
    cv2.destroyAllWindows()


def threadVideoShow(source=0):
    """
    Dedicated thread for grabbing video frames with VideoShow object.
    Main thread serves to Object[Cup] detection using Yolov8.
    """

    cap = cv2.VideoCapture(source)
    (grabbed, frame) = cap.read()
    video_shower = VideoShow(frame).start()
    cps = CountsPerSec().start()
    # video_getter = VideoGet(source).start()
    # YOLOv8n is the fastest option
    # See github.com/ultralytics/ultralytics
    model = YOLO("yolov8n.pt")
    # model.info()
    # ClassNames List [0:79]
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
                  "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                  "teddy bear", "hair drier", "toothbrush"
                  ]

    while True:
        (grabbed, frame) = cap.read()
        if not grabbed or video_shower.stopped:
            video_shower.stop()
            break

        frame = putIterationsPerSec(frame, cps.counts_per_sec())
        # video_shower.frame = frame
        cps.increment()
        # Object detection using YOLOv8, frame by frame
        # Classes=41 to filter Cup Class
        # Adjust Confidence Interval
        # Stream=True to generate a memory-efficient generator of Results objects.
        results = model(frame, stream=True, conf=0.3, classes=41)
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                # print(x1, y1, x2, y2)

                # Draw the bounding box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Calculate confidence value
                conf = math.ceil((box.conf[0] * 100)) / 100

                # Get the class number and class name
                cls = int(box.cls[0])
                class_name = classNames[cls]

                # Write the class name and confidence value
                label = f"{class_name}{conf}"
                text_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                c2 = x1 + text_size[0], y1 - text_size[1] - 3
                cv2.rectangle(frame, (x1, y1), c2, [255, 0, 0], -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, y1 - 2), 0,
                            1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
            resized_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            # cv2.imshow("Video", resized_frame)
            video_shower.frame = resized_frame

    cap.release()
    cv2.destroyAllWindows()


def threadVideoBoth(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Dedicated thread for showing video frames with VideoShow object.
    Main thread serves to pass frames between VideoGet and
    VideoShow objects/threads and to Object[Cup] detection using Yolov8.
    """

    video_getter = VideoGet(source).start()
    video_shower = VideoShow(video_getter.frame).start()
    cps = CountsPerSec().start()
    # video_getter = VideoGet(source).start()
    # YOLOv8n is the fastest option
    # See github.com/ultralytics/ultralytics
    model = YOLO("yolov8n.pt")
    # model.info()
    # ClassNames List [0:79]
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
                  "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                  "teddy bear", "hair drier", "toothbrush"
                  ]

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.counts_per_sec())
        # video_shower.frame = frame
        cps.increment()
        # Object detection using YOLOv8, frame by frame
        # Classes=41 to filter Cup Class
        # Adjust Confidence Interval
        # Stream=True to generate a memory-efficient generator of Results objects.
        results = model(frame, stream=True, conf=0.3, classes=41)
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                # print(x1, y1, x2, y2)

                # Draw the bounding box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Calculate confidence value
                conf = math.ceil((box.conf[0] * 100)) / 100

                # Get the class number and class name
                cls = int(box.cls[0])
                class_name = classNames[cls]

                # Write the class name and confidence value
                label = f"{class_name}{conf}"
                text_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
                c2 = x1 + text_size[0], y1 - text_size[1] - 3
                cv2.rectangle(frame, (x1, y1), c2, [255, 0, 0], -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, y1 - 2), 0,
                            1, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA)
            resized_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            # cv2.imshow("Video", resized_frame)
            video_shower.frame = resized_frame

    video_getter.stop()
    cv2.destroyAllWindows()


def thread_Final(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    VideoShow object is extracted.
    Main thread serves to Object[Cup] detection using Yolov8.
    Simplification in box labeling.
    """

    video_getter = VideoGet(source).start()
    cps = CountsPerSec().start()
    # YOLOv8n is the fastest option
    # See github.com/ultralytics/ultralytics
    model = YOLO("yolov8n.pt")
    classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
                  "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                  "teddy bear", "hair drier", "toothbrush"
                  ]

    while cv2.waitKey(1) != 27:  # ESC to exit
        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.counts_per_sec())
        cps.increment()
        # Object detection using YOLOv8, frame by frame
        # Classes=41 to filter Cup Class
        # Adjust Confidence Interval
        # Stream=True to generate a memory-efficient generator of Results objects.
        results = model(frame, stream=True, conf=0.5, classes=41)
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                # print(x1, y1, x2, y2)

                # Draw the bounding box
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Calculate confidence value
                # conf = math.ceil((box.conf[0] * 100)) / 100
                # Confidence value (avoid conversion if possible)
                conf = box.conf[0]  # Assuming direct confidence access

                # Get the class number and class name
                cls = int(box.cls[0])
                class_name = classNames[cls]

                # Write the class name and confidence value
                label = f"{class_name}{conf:.2f}"  # Use f-string formatting for efficiency
                # text_size = class_text_sizes[class_name]
                # c2 = x1 + text_size[0], y1 - text_size[1] - 3
                # cv2.rectangle(frame, (x1, y1), c2, [255, 0, 0], -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, y1 - 2), 0,
                            1, [0, 255, 0], thickness=2, lineType=cv2.LINE_AA)
            resized_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
            cv2.imshow("Video", resized_frame)

    video_getter.stop()
    cv2.destroyAllWindows()


def main():
    # noThreading()  # 6 iterations per second / 29 iterations per second [with / without Yolov8]
    # threadVideoGet()  # 6 iterations per second / 400 iterations per second [with / without Yolov8]
    # threadVideoShow()  # 4 iterations per second / 30 iterations per second [with / without Yolov8]
    # threadVideoBoth()  # 4 iterations per second / 40K iterations per second [with / without Yolov8]
    thread_Final()


if __name__ == "__main__":
    main()
