#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys

from baxter_interface import Limb

import helpers 
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
# from geographic_msgs import Transform
# from geographic_msgs import Vector3
# from geographic_msgs import Quaternion

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

import transformations

import tf2_ros

import tf2_msgs.msg
import geometry_msgs.msg

# Topic name to publish
FRAME_TOPIC = "/tf"

FRAME_MSG_TYPE = tf2_msgs.msg.TFMessage
    
def main(args):
    """
    Main Script
    """

    tag_frame = args[1]
    stick_frame = args[2]
    left_hand_frame = args[3]
    claw_frame = args[4]
    right_hand_frame = args[5]

    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)

    frame_pub = rospy.Publisher(FRAME_TOPIC, FRAME_MSG_TYPE, queue_size=10)

    stick_planner = PathPlanner("left_arm")
    claw_planner = PathPlanner("right_arm")

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                while(True):
                    #============================================
                    # GET TAG TO STICK
                    tag_to_claw_target_trans = np.array([0, 0, 0])
                    tag_to_claw_target_rot = np.array([0.7071068, 0.7071068, 0, 0])
                    tag_to_claw_target_t = helpers.vec_to_g(tag_to_claw_target_trans, tag_to_claw_target_rot)

                    #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
                    tag_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", tag_frame, rospy.Time(0)))

                    #GET STICK TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
                    claw_target_t = np.matmul(tag_t, tag_to_claw_target_t)
                    #============================================
                    # PUBLISH TO TF
                    
                    frame_pub.publish(helpers.g_to_tf(claw_target_t, "base", "claw_target"))
                    #============================================

                    claw_to_right_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(claw_frame, right_hand_frame, rospy.Time(0)))

                    right_hand_target_t = np.matmul(claw_target_t, claw_to_right_hand_t)

                    #CREATE HAND TARGET POSE     
                    right_hand_target_pose = helpers.g_to_pose(right_hand_target_t, "base")
                    #============================================
                    # PUBLISH TO TF

                    frame_pub.publish(helpers.g_to_tf(right_hand_target_t, "base", "right_hand_target"))
                    #============================================

                    if raw_input("Press q to calculate path to align to tag, anything else to skip: ") == "q":

                        # GENERATE AND EXECUTE PLAN
                        plan = claw_planner.plan_to_pose(right_hand_target_pose, [])

                        print(plan)

                        if raw_input("Press q to execute plan, anything else to skip: ") == "q":

                            if not claw_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                #============================================
                while(True):
                    
                    if raw_input("Press q to plan movement down, anything else to skip: ") == "q":

                        
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", left_hand_frame, rospy.Time(0)))

                        move_down_trans = np.array([0, 0, -.015])
                        move_down_t = transformations.translation_matrix(move_down_trans)

                        hand_target_t = np.matmul(move_down_t, hand_t) #move down in spatial frame

                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")

                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move down, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan movement forward, ahything else to skip: ") == "q":

                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", left_hand_frame, rospy.Time(0)))

                        move_forward_trans = np.array([0, 0, .0375])
                        move_forward_t = transformations.translation_matrix(move_forward_trans)

                        hand_target_t = np.matmul(hand_t, move_forward_t) #move forward in hand frame
                        
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")

                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move forward, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan motion of right arm to block, anything else to skip: ") == "q":

                        stick_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", stick_frame, rospy.Time(0)))

                        stick_to_claw_target_trans = np.array([0, 0, -0.065])
                        stick_to_claw_target_rot = np.array([1, 0, 0, 0])
                        stick_to_claw_target_t = helpers.vec_to_g(stick_to_claw_target_trans, stick_to_claw_target_rot)

                        claw_target_t = np.matmul(stick_t, stick_to_claw_target_t)

                        frame_pub.publish(helpers.g_to_tf(claw_target_t, "base", "claw_target"))

                        claw_to_right_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(claw_frame, right_hand_frame, rospy.Time(0)))

                        right_hand_target_t = np.matmul(claw_target_t, claw_to_right_hand_t)     

                        frame_pub.publish(helpers.g_to_tf(right_hand_target_t, "base", "right_hand_target"))

                        right_hand_target_pose = helpers.g_to_pose(right_hand_target_t, "base")

                        plan = claw_planner.plan_to_pose(right_hand_target_pose, [])
                        
                        if raw_input("Press q to move right hand, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                    raise Exception("Execution failed")

            except Exception as e:
                print(e)
                traceback.print_exc()

  

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main(sys.argv)
