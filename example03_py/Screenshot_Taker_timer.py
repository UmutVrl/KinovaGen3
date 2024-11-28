##########################################################
#    Kinova Gen3 Robotic Arm                             #
#                                                        #
#    Taking timed screenshots from the camera            #
#                                                        #
#                                                        #
#    written by: U. Vural                                #
#                                                        #
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
import time

# Import the utilities helper module of Kinova
import argparse
import utilities


def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Camera Streaming via Ethernet
        source = cv2.VideoCapture("rtsp://192.168.1.10/color")
        source.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        count = 0
        frame_width = 1280
        frame_height = 720
        write_path = os.getcwd() + "/resources/calibration_screenshots_snickers_"

        # Initialize the time tracker
        last_capture_time = time.time()

        while cv2.waitKey(1) != 27:  # press ESC to exit
            has_frame, frame = source.read()
            frame = cv2.resize(frame, (frame_width, frame_height), 3)
            # Note: Be careful with frame dimensions. These have to match with the main pick&place program.

            if has_frame:
                # print("Frame Shape", frame.shape)
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                cv2.imshow("Output Video", frame)
                # cv2.imshow("Grayscale Video", frame_gray)
                if time.time() - last_capture_time > 3:  # More than 5 seconds have passed
                    cv2.imwrite(write_path + str(count) + ".jpg", frame)
                    cv2.rectangle(frame, (0, frame_height//3), (frame_width, frame_height//2), (0, 255, 0), cv2.FILLED)
                    cv2.putText(frame, "Scan Saved", (frame_width//2-120, frame_height//2-40),
                                cv2.FONT_HERSHEY_DUPLEX, 2, (255, 0, 0), 3)
                    cv2.imshow("Output Video", frame)
                    print("Scan Saved. Frame Shape:", frame.shape)
                    cv2.waitKey(40)  # Waiting duration between each screenshot.
                    # Take many screenshots with different position & angle combinations for better precision (100+)
                    count += 1
                    last_capture_time = time.time()  # Reset the last capture time
            else:
                print("No frame")
                break

        source.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
