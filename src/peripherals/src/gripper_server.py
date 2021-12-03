#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

# push_peripheral.py node reads incoming serial data and converts
# the data into float values, which represent the force applied to
# a jenga block. It publishes the float values to the topic: push_force

import rospy
import serial
from gripper import Gripper
from peripherals.srv import GripperSrv


def gripper_service():
    rospy.init_node('gripper_server', anonymous=True)
    g = Gripper()
    service = rospy.Service('gripper_srv', GripperSrv, g.request_handler)
    rospy.spin()
        

if __name__ == '__main__':
    try:
        gripper_service()
    except rospy.ROSInterruptException:
        pass