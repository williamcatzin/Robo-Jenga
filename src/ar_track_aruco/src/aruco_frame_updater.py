#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import geometry_msgs
from scipy.spatial.transform import Rotation as R

CAMERA_MATRIX = np.array([[406.167596765, 0.0, 649.310826191], 
                          [0.0, 406.167596765, 412.393413372], 
                          [0.0, 0.0, 1.0]])

DISTORTION_COEFFS = np.array([0.0194573814166, -0.0580583678532, 0.00267653694509, 0.000415440329517, 0.0159403471267])

CAMERA_TOPIC = "/cameras/left_hand_camera/image"

IMAGE_MSG_TYPE = Image

FRAME_TOPIC = "/tf"

FRAME_MSG_TYPE = tf2_msgs.msg.TFMessage

CAMERA_FRAME = "left_hand_camera"

TAG_FRAME = "aruco_tag_{}"

class frame_updater:

    def __init__(self):
        self.camera_matrix = CAMERA_MATRIX #TODO: change to params
        self.distortion_coeffs = DISTORTION_COEFFS #TODO: change to params

        self.image_pub = rospy.Publisher("/cvimage", Image, queue_size=10) #TODO: get from params
        self.frame_pub = rospy.Publisher(FRAME_TOPIC, FRAME_MSG_TYPE, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, IMAGE_MSG_TYPE, self.callback) #TODO: get from params

    def callback(self,image_msg):
        try:
            # Convert ROS image to CV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # Change from RGB to HSV (Hue, Saturation, Value)
            cv_grayImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Lower bound on gray scale
            # lower_gray = np.array([0, 5, 50])
            # Upper bound on gray scale
            # upper_gray = np.array([179, 50, 255])

            # the bounds above are producing an error with converting the image
            # to gray scale in cv

            

            # Filter image using desired gray scale
            # mask = cv2.inRange(hsv, lower_gray, upper_gray)
            # Mask cv_image with filter to get only paper with ar marker
            # masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # cv2.imshow("window", cv_grayImage)
            # cv2.waitKey(3)




            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Use 4X4 dictionary to find markers
            detector_parameters = aruco.DetectorParameters_create()  # Marker detection parameters
            # lists of ids and the corners beloning to each id
            corners, ids, rejected_img_points = aruco.detectMarkers(cv_grayImage, aruco_dict,
                                                                parameters=detector_parameters,
                                                                cameraMatrix=self.camera_matrix,
                                                                distCoeff=self.distortion_coeffs)

            if np.all(ids is not None):
                for i in range(len(ids)):
                    id = ids[i]
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.camera_matrix, self.distortion_coeffs)
                    print("FRAME-UPDATE:  TAG {}:\nTVEC:\n{}\nRVEC:\n{}\n".format(id, tvec, rvec))
                    aruco.drawDetectedMarkers(cv_grayImage, corners)  # Draw A square around the markers
                    aruco.drawAxis(cv_grayImage, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 0.01)  # Draw Axis
                    
                    tvec = np.array([tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]])
                    rvec = np.array([rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]])

                    r = R.from_rotvec(rvec)
                    q = r.as_quat()
                    
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.frame_id = CAMERA_FRAME
                    t.header.stamp = rospy.Time.now()
                    t.child_frame_id = TAG_FRAME.format(id)
                    t.transform.translation.x = tvec[0]
                    t.transform.translation.y = tvec[1]
                    t.transform.translation.z = tvec[2]

                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]


                    tfm = tf2_msgs.msg.TFMessage([t])
                    self.frame_pub.publish(tfm)

            topic_image = self.bridge.cv2_to_imgmsg(cv_grayImage)
            self.image_pub.publish(topic_image)

        except CvBridgeError as e:
            print(e)
