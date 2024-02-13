##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Color detection (Part 3)                            #
#    Color Calibration from the taken screenshot         #
#                                                        #
#    See also:                                           #
#    02b_color_detection_part1 for taking screenshot     #
#    02b_color_detection_part2 for threshold calibration #
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
#                                                        #
#                                                        #
##########################################################

import cv2

image = cv2.imread("resources/calibration_screenshot1.jpg")
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# (from 02b_color_detection_part2:)
# Color: h_min h_max s_min s_max v_min v_max
# Green: 50 70 166 255 14 255
# Blue: 110 119 206 255 156 255
# Yellow: 13 22 221 255 147 2555
# Red:0 12 210 255 141 255

lower_range = (13, 221, 147)  # HRV min
upper_range = (22, 255, 255)  # MRV max

mask = cv2.inRange(hsv_image, lower_range, upper_range)

color_image = cv2.bitwise_and(image, image, mask=mask)

cv2.imshow("Detected Color:", color_image)

cv2.waitKey(0)