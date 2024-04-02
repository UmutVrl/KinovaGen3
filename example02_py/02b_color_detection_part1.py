##########################################################
#    Kinova Gen3 Robotic Arm                             #
#    Color Detection (Part 1)                            #
#    Taking a screenshot from the camera                 #
#                                                        #
#    See also:                                           #
#    02b_color_detection_part2 for threshold calibration #
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
import os

source = cv2.VideoCapture("rtsp://192.168.1.10/color")
count = 0

write_path = os.getcwd() + "/resources/calibration_screenshot"

while True:
    ret, frame = source.read()
    frame = cv2.resize(frame, (640, 480), 3)

    if not ret or cv2.waitKey(1) == ord("q"):  # press q to exit
        break

    if ret:
        # print("Frame Shape", frame.shape)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow("Output video", frame)
        cv2.imshow("grayscale video", frame_gray)
        if cv2.waitKey(1) & 0xFF == ord("s"):  # press s to take screenshot
            cv2.imwrite(write_path + str(count) + ".jpg", frame)
            cv2.rectangle(frame, (0, 200), (640, 300), (0, 255, 0), cv2.FILLED)
            cv2.putText(frame, "Scan Saved", (150, 265), cv2.FONT_HERSHEY_DUPLEX, 2, (255, 0, 0), 2)
            cv2.imshow("Output video", frame)
            print("Frame Shape", frame.shape)
            cv2.waitKey(30)
            count += 1
    else:
        break

source.release()
cv2.destroyAllWindows()