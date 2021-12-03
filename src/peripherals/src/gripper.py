#!/usr/bin/env python
import rospy
import serial
from peripherals.srv import GripperSrv

# Port name for serial port
_port = '/dev/ttyUSB0'
# baude rate of the esp32
_baudrate = 9600

class Gripper:
    def request_handler(self, request):
      if request.command == "open":
        self.open_gripper()
      elif request.command == "close":
        self.close_gripper()

    #When another node calls the service, return the last image
    def open_gripper(self):
      ser = serial.Serial(_port, _baudrate)
      command = 'O'
      ser.write(command.encode())
      data = ser.readline().decode('utf-8')
      print(data)
      return "Finished"

    def close_gripper(self):
      ser = serial.Serial(_port, _baudrate)
      command = 'C'
      ser.write(command.encode())
      data = ser.readline().decode('utf-8')
      print(data)
      return "Finished"