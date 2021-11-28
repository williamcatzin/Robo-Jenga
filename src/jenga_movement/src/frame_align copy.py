#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
assert sys.argv[1] in ("baxter")
ROBOT = sys.argv[1]

from baxter_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

import transformations

import tf2_ros
    
def main(args):
    """
    Main Script
    """

    tag_frame = args[1]
    tool_frame = args[2]

    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("left_arm")

    Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
    Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])


    limb = Limb("left")
    controller = Controller(Kp, Ki, Kd, Kw, limb)


    ##
    ## Add the obstacle to the planning scene here
    ##

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper"
    # orien_const.header.frame_id = "base"
    # orien_const.orientation.y = -1.0
    # orien_const.absolute_x_axis_tolerance = 0.1
    # orien_const.absolute_y_axis_tolerance = 0.1
    # orien_const.absolute_z_axis_tolerance = 0.1
    # orien_const.weight = 1.0

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                
              #define transform from tag to tool target (static for now)
              tag_to_tool_target_trans = np.array([0, 0, 0])
              tag_to_tool_target_rot = np.array([0, 1, 0, 0])

              tag_to_tool_target_t = np.matmul(transformations.quaternion_matrix(tag_to_tool_target_rot), transformations.translation_matrix(tag_to_tool_target_trans))

              #tool_target = tag * tag_to_target
              (tag_trans,tag_rot) = tfBuffer.lookupTransform('/base', tag_frame, rospy.Time(0))
              tag_t = np.matmul(transformations.quaternion_matrix(tag_rot), transformations.translation_matrix(tag_trans))

              tool_target_t = np.matmul(tag_to_tool_target_t, tag_t)

              #hand_target = tool_to hand * tool_target (use static defined transform for tool_to_hand)

              hand_target_pose = PoseStamped()
              hand_target_pose.header.frame_id = "base"

              #x, y, and z position
              hand_target_pose.pose.position.x = x
              hand_target_pose.pose.position.y = y
              hand_target_pose.pose.position.z = z

              #Orientation as a quaternion
              hand_target_pose.pose.orientation.x = 0.0
              hand_target_pose.pose.orientation.y = -1.0
              hand_target_pose.pose.orientation.z = 0.0
              hand_target_pose.pose.orientation.w = 0.0

              # Might have to edit this . . . 
              plan = planner.plan_to_pose(hand_target_pose, [])

              raw_input("Press <Enter> to move the right arm to goal pose 1: ")
              if not controller.execute_path(plan):
                  raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main(sys.argv)
