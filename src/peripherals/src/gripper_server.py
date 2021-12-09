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
from peripherals.srv import GripperSrv


# Port name for serial port
_port = '/dev/ttyUSB1'
# baude rate of the esp32
_baudrate = 9600

#When another node calls the service, return the last image
def close_gripper(msg):
    print("Requested Close Gripper Service")
    ser = serial.Serial(_port, _baudrate)
    command = 'P'
    ser.write(command.encode())
    return "Finished"

def open_gripper(msg):
    print("Requested Open Gripper Service")
    ser = serial.Serial(_port, _baudrate)
    command = 'D'
    ser.write(command.encode())
    return "Finished"


def gripper_service():
    rospy.init_node('gripper_server', anonymous=True)
    rospy.Service('close_gripper', GripperSrv, close_gripper)
    rospy.Service('open_gripper', GripperSrv, open_gripper)
    rospy.spin()
        

if __name__ == '__main__':
    try:
        gripper_service()
    except rospy.ROSInterruptException:
        pass