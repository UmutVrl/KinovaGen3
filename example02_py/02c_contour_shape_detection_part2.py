##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Contour & Shape detection (Part 2)                  #
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
original_image_copy = image.copy()
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
lower_threshold = 10
upper_threshold = 60
canny_edge = cv2.Canny(image_grayscale, lower_threshold, upper_threshold)

# contour detection
contours, hierarchy = cv2.findContours(canny_edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# length of the contour
print(f"Number of Contours = {str(len(contours))}")

for cnt in contours:
    # area of the contour
    area = cv2.contourArea(cnt)
    print("Area of the contour", area)

    # draw contour
    cv2.drawContours(original_image_copy, contours, -1, (0, 255, 0), 2)

    # arc length of the contour
    peri = cv2.arcLength(cnt, True)
    # corner points of the shape
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    object_corner = len(approx)
    print(f"Number of the corner points", object_corner)

    # Draw bounding boxes around each of the shape
    x, y, w, h = cv2.boundingRect(approx)
    cv2.rectangle(original_image_copy, (x-10, y-10), (x + w + 10, y + h + 10), (0, 0, 0), 1)

    if object_corner == 3:
        object_type = "triangle"
    elif object_corner == 4:
        object_type = "quadrangle"
    elif object_corner > 4:
        object_type = "oval"
    else:
        object_type = "none"

    cv2.putText(original_image_copy, object_type,
                (x - 15, y - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 0), 1)

cv2.imshow("Original Image", image)
cv2.imshow("Color Detection", color_image)
# cv2.imshow("Grayscale Image", image_grayscale)
cv2.imshow("Edge Detector", canny_edge)
cv2.imshow("Final Output", original_image_copy)

cv2.waitKey(0)
