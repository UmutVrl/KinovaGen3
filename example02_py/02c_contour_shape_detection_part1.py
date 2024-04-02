##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Contour & Shape detection (Part 1)                  #
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

# convert BGR image to HSV
image = cv2.imread("resources/calibration_screenshot0.jpg")
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# (data from 02b_color_detection_part2)
# Color: h_min h_max s_min s_max v_min v_max
# Green: 50 70 166 255 14 255
# Blue: 110 119 206 255 156 255
# Yellow: 13 22 221 255 147 2555
# Red:0 12 210 255 141 255
lower_range = (50, 166, 14)  # HRV min
upper_range = (70, 255, 255)  # MRV max
mask = cv2.inRange(hsv_image, lower_range, upper_range)
color_image = cv2.bitwise_and(image, image, mask=mask)

# convert image to gray scale
image_grayscale = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

# appy canny edge detector to detect edges
lower_threshold = 100
upper_threshold = 150
canny_edge = cv2.Canny(image_grayscale, lower_threshold, upper_threshold)

# contour detection
contours, hierarchy = cv2.findContours(canny_edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# length of the contour
print(f"Number of Contours = {str(len(contours))}")

# draw contour
original_image_copy = image.copy()

cv2.drawContours(original_image_copy, contours, -1, (0, 255, 0), 3)

cv2.imshow("Original Image", image)
cv2.imshow("Color Detection", color_image)
cv2.imshow("Grayscale Image", image_grayscale)
cv2.imshow("Edge Detector", canny_edge)
cv2.imshow("Draw Contours", original_image_copy)


cv2.waitKey(0)




