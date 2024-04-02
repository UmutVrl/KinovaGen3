##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Color detection (Part 2)                            #
#    Threshold Calibration from the taken screenshot     #
#                                                        #
#    See also:                                           #
#    02b_color_detection_part1 for taking screenshot     #
#    02b_color_detection_part3 for color detection       #
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

import cv2
import numpy as np


def empty(a):
    pass


cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars", 640, 240)

# https://docs.opencv.org/3.2.0/df/d9d/tutorial_py_colorspaces.html
cv2.createTrackbar("Hue Min", "Trackbars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "Trackbars", 179, 179, empty)
cv2.createTrackbar("Sat Min", "Trackbars", 0, 255, empty)
cv2.createTrackbar("Sat Max", "Trackbars", 255, 255, empty)
cv2.createTrackbar("Val Min", "Trackbars", 0, 255, empty)
cv2.createTrackbar("Val Max", "Trackbars", 255, 255, empty)

while True:

    if cv2.waitKey(1) == ord("q"):  # press q to exit
        break

    image = cv2.imread("resources/calibration_screenshot0.jpg")
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
    h_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
    s_min = cv2.getTrackbarPos("Sat Min", "Trackbars")
    s_max = cv2.getTrackbarPos("Sat Max", "Trackbars")
    v_min = cv2.getTrackbarPos("Val Min", "Trackbars")
    v_max = cv2.getTrackbarPos("Val Max", "Trackbars")

    print("h_min:{} h_max:{} s_min:{} s_max:{} v_min:{} v_max:{}".format(h_min, h_max, s_min, s_max, v_min, v_max))
    # Color: h_min h_max s_min s_max v_min v_max
    # Green : 50 70 166 255 14 255
    # Blue : 110 119 206 255 156 255
    # Yellow : 13 22 221 255 147 2555
    # Red : 0 12 210 255 141 255
    # IMPORTANT: These values should be different for each screenshot.
    # It would be wise to re-apply threshold values if that is the case

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(image_HSV, lower, upper)

    cv2.imshow("Original Image", image)
    cv2.imshow("HSV Image", image_HSV)
    cv2.imshow("Mask Image", mask)

    cv2.waitKey(1)

cv2.destroyAllWindows()