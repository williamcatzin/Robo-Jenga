#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

# push_peripheral.py node reads incoming serial data and converts
# the data into float values, which represent the force applied to
# a jenga block. It publishes the float values to the topic: push_force

import rospy
from std_msgs.msg import Float32
import serial

# Port name for serial port
_port = '/dev/ttyUSB0'
# baude rate of the esp32
_baudrate = 460800

def main():
    rospy.init_node('push_peripheral', anonymous=True)
    pub = rospy.Publisher('push_force', Float32, queue_size=10)
    ser = serial.Serial(_port, _baudrate)
    ser.flushInput()
    ser.flushOutput()
    rate = rospy.Rate(5) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        data = float(ser.readline().decode('utf-8'))
        # Uncomment to print load cell data
        print(data)
        pub.publish(data)
        rate.sleep()
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass