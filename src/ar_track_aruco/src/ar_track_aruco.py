#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

import rospy
import tf2_ros
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from aruco_frame_updater import frame_updater

def main(args):
    topic_name = args[1]
    frame = args[2]
    ar_marker_size = args[3] 
    rospy.init_node('ar_track_aruco', anonymous=True)
    fu = frame_updater(topic_name, frame, ar_marker_size)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # sys.argv contains camera topics
    main(sys.argv)