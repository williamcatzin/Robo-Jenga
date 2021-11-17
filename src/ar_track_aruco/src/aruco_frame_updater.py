#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

CAMERA_MATRIX = np.array([[406.167596765, 0.0, 649.310826191], 
                          [0.0, 406.167596765, 412.393413372], 
                          [0.0, 0.0, 1.0]])

DISTORTION_COEFFS = np.array([0.0194573814166, -0.0580583678532, 0.00267653694509, 0.000415440329517, 0.0159403471267])

CAMERA_TOPIC = "/cameras/left_hand_camera/image"

IMAGE_MSG_TYPE = Image

class frame_updater:

    def __init__(self):
        self.camera_matrix = CAMERA_MATRIX #TODO: change to params
        self.distortion_coeffs = DISTORTION_COEFFS #TODO: change to params

        self.image_pub = rospy.Publisher("/cvimage", Image, queue_size=10) #TODO: get from params

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, IMAGE_MSG_TYPE, self.callback) #TODO: get from params

    def callback(self,image_msg):
        try:
            #print("{}, {}".format(image_msg.height, image_msg.width))
            frame = np.array(self.bridge.imgmsg_to_cv2(image_msg, "bgr8"))
            #print(frame.shape)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Use 4X4 dictionary to find markers
            detector_parameters = aruco.DetectorParameters_create()  # Marker detection parameters
            # lists of ids and the corners beloning to each id
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=detector_parameters,
                                                                cameraMatrix=self.camera_matrix,
                                                                distCoeff=self.distortion_coeffs)

            if np.all(ids is not None):
                for i in range(len(ids)):
                    id = ids[i]
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.camera_matrix, self.distortion_coeffs)
                    rot_matrix = np.empty((3,3))
                    cv2.Rodrigues(rvec, dst=rot_matrix)
                    print("TAG {}:\nTVEC:\n{}\nRVEC:\n{}\nRMAT:\n{}\n".format(id, tvec, rvec, rot_matrix))
                    aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    aruco.drawAxis(frame, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 0.01)  # Draw Axis
                    # Display the resulting frame
            
            #cv2.imshow('frame', frame)
            topic_image = self.bridge.cv2_to_imgmsg(frame)
            self.image_pub.publish(topic_image)

        except CvBridgeError as e:
            print(e)
