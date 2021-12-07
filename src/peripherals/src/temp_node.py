#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

# push_peripheral.py node reads incoming serial data and converts
# the data into float values, which represent the force applied to
# a jenga block. It publishes the float values to the topic: push_force

import sys
import rospy
from peripherals.srv import GripperSrv

def comm_gripper(comm):
    # rospy.wait_for_service('open_gripper')
    # try:
    #     send_command = rospy.ServiceProxy('open_gripper', GripperSrv)
    #     response = send_command("D")
    #     print(response)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)
    
    rospy.wait_for_service('close_gripper')
    try:
        send_command = rospy.ServiceProxy('close_gripper', GripperSrv)
        response = send_command("D")
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == '__main__':
    try:
        comm_gripper("P")
    except rospy.ROSInterruptException:
        pass

