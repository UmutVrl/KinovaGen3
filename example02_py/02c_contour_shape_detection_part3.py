##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Contour & Shape detection (Part 3)                  #
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
# Import the utilities module for Kinova
import argparse
import utilities


def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        source = cv2.VideoCapture("rtsp://192.168.1.10/color")
        win_name = 'Camera Preview'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 640, 360)

        while cv2.waitKey(1) != 27:  # ESC to exit

            has_frame, frame = source.read()

            if not has_frame:
                break

            original_frame_copy = frame.copy()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # (data from 02b_color_detection_part2)
            # Color: h_min h_max s_min s_max v_min v_max
            # Green: 50 70 166 255 14 255
            # IMPORTANT: These values should be different for each screenshot.
            # It would be wise to re-apply threshold values if that is the case
            # see 02b_color_detection_part2

            lower_range = (30, 126, 14)  # HRV min
            upper_range = (80, 255, 255)  # MRV max
            mask = cv2.inRange(hsv_frame, lower_range, upper_range)
            color_frame = cv2.bitwise_and(frame, frame, mask=mask)

            # convert image to gray scale
            grayscale_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

            # remove noise
            # https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html
            kernel = np.ones((9, 9), np.uint8)
            # erosion = cv2.erode(image_grayscale, kernel, iterations=1)
            # dilation = cv2.dilate(erosion, kernel, iterations=1)
            opening = cv2.morphologyEx(grayscale_frame, cv2.MORPH_OPEN, kernel)

            # apply canny edge detector to detect edges
            lower_threshold = 10
            upper_threshold = 90
            canny_edge = cv2.Canny(opening, lower_threshold, upper_threshold)

            # threshold image
            # ret, thresh_binary = cv2.threshold(canny_edge.copy(), 125, 1, cv2.THRESH_BINARY)

            # contour detection
            contours, hierarchy = cv2.findContours(canny_edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # number of contour
            print(f"Number of Contours = {str(len(contours))}")

            for cnt in contours:
                # area of the contour
                area = cv2.contourArea(cnt)
                print("Area of the contour", area)

                # if the size of the contour is greater than a threshold
                if area >= float(100):

                    # draw contour
                    cv2.drawContours(original_frame_copy, contours, -1, (0, 255, 0), 2)

                    # arc length of the contour
                    peri = cv2.arcLength(cnt, True)
                    # corner points of the shape
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    object_corner = len(approx)
                    print(f"Number of the corner points", object_corner)

                    # Draw bounding boxes around each of the shape
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(original_frame_copy, (x - 10, y - 10), (x + w + 10, y + h + 10), (0, 0, 0), 1)

                    # simple naming
                    if object_corner == 3:
                        object_type = "triangle"
                    elif object_corner == 4:
                        object_type = "quadrangle"
                    elif object_corner > 4:
                        object_type = "oval"
                    else:
                        object_type = "none"

                    cv2.putText(original_frame_copy, object_type,
                                (x - 15, y - 15), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 0), 1)

            cv2.imshow("Original Image", frame)
            cv2.imshow("Color Detection", color_frame)
            cv2.imshow("Grayscale Image", grayscale_frame)
            cv2.imshow("Edge Detector", canny_edge)
            cv2.imshow("Final Output", original_frame_copy)
            cv2.waitKey(1)

        source.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
