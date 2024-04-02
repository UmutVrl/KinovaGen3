##########################################################
#    Camera access for Kinova Gen3 Robotic Arm           #
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


# Libraries
import cv2

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

        print(cv2.getBuildInformation())
        # print(cv2.__file__)

        win_name = 'Camera Preview'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        while cv2.waitKey(1) != 27:  # press ESC to exit
            has_frame, frame = source.read()
            if not has_frame:
                break
            cv2.imshow(win_name, frame)

        source.release()
        cv2.destroyWindow(win_name)


if __name__ == "__main__":
    main()




