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
from path_planner import PathPlanner
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

#Define the method which contains the main functionality of the node.
def measure(frame1, frame2):
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

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      # Get the transform between the left camera and the target ar marker
      trans = tfBuffer.lookup_transform(frame1, frame2, rospy.Time())
      # Build Path Planner Object
      planner = PathPlanner("left_arm")

      # Create a path constraint for the arm
      # UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
      orien_const = OrientationConstraint()
      orien_const.link_name = "left_gripper"
      orien_const.header.frame_id = "base"
      orien_const.orientation.y = -1.0
      orien_const.absolute_x_axis_tolerance = 0.1
      orien_const.absolute_y_axis_tolerance = 0.1
      orien_const.absolute_z_axis_tolerance = 0.1
      orien_const.weight = 1.0

      while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                x, y, z = 0.47, -0.85, 0.07
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [orien_const])

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


def main():
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('ar_test', anonymous=True)

  try:
    if not rospy.has_param("~frame/1"):
        return False
    frame1 = rospy.get_param("~frame/1")
    if not rospy.has_param("~frame/2"):
        return False
    frame2 = rospy.get_param("~frame/2")

    measure(frame1, frame2)
  except rospy.ROSInterruptException:
    pass

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  main()
