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
import math

from geometry_msgs.msg import Twist

#Define the method which contains the main functionality of the node.
def align(tool_frame, target_tool_frame):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  tfBuffer = tf2_ros.Buffer()
  _ = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # get tool frame (offset from wrist) in launch file

  # get target frame (offset from ar tag) in launch file

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(tool_frame, target_tool_frame, rospy.Time())

      #print(trans)
      print(math.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2 + trans.transform.translation.z**2))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


def main(args):
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  tool_frame = args[1]

  tool_target_frame = args[2]

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('ar_test', anonymous=True)

  try:
    align(tool_frame, tool_target_frame)
  except rospy.ROSInterruptException:
    pass

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  main(sys.argv)
