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
    tool_frame = args[2]
    hand_frame = args[3]
    claw_frame = args[4]
    right_hand_frame = args[5]

    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)

    frame_pub = rospy.Publisher(FRAME_TOPIC, FRAME_MSG_TYPE, queue_size=10)

    # Make sure that you've looked at and understand path_planner.py before starting

    stick_planner = PathPlanner("left_arm")
    claw_planner = PathPlanner("right_arm")

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
                # ar tag's current position frame to tool target frame
                #define transform from tag to tool target (static for now)
                tag_to_tool_target_trans = np.array([0, 0, -.05])
                tag_to_tool_target_rot = np.array([0.7071068, 0.7071068, 0, 0])
                tag_to_tool_target_t = helpers.vec_to_g(tag_to_tool_target_trans, tag_to_tool_target_rot)

                #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
                tag_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", tag_frame, rospy.Time(0)))

                #GET TOOL TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
                tool_target_t = np.matmul(tag_t, tag_to_tool_target_t)


                #============================================

                # PUBLISH TO RVIZ
                
                frame_pub.publish(helpers.g_to_tf(tool_target_t))
                #============================================

                # (THIS IS CONSTANT) tool frame to hand frame
                #hand_target = tool_to hand * tool_target (tool_to_hand is defined in a static transform, so it should always be the same)
                tool_to_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(tool_frame, hand_frame, rospy.Time(0)))

                hand_target_t = np.matmul(tool_target_t, tool_to_hand_t)

                #CREATE HAND TARGET POSE     
                hand_target_pose = helpers.g_to_pose(hand_target_t)
                #============================================

                #RVIZ STUFF
                frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                #============================================

                if raw_input("Press q to calculate path to align to tag, anything else to skip: ") == "q":

                    # GENERATE AND EXECUTE PLAN
                    plan = stick_planner.plan_to_pose(hand_target_pose, [])

                    print(plan)

                    if raw_input("Press q to execute plan, anything else to skip: ") == "q":

                        if not stick_planner.execute_plan(plan):
                            raise Exception("Execution failed")

                #============================================

                #add for loop for each vertical offset

                while(True):
                    
                    if raw_input("Press q to plan movement down, anything else to skip: ") == "q":

                        
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", hand_frame, rospy.Time(0)))

                        move_down_trans = np.array([0, 0, -.015])
                        move_down_t = transformations.translation_matrix(move_down_trans)

                        hand_target_t = np.matmul(move_down_t, hand_t) #move down in spatial frame

                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                        hand_target_pose = helpers.g_to_pose(hand_target_t)

                        # Might have to edit this . . . 
                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move down, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan movement forward, ahything else to skip: ") == "q":

                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", hand_frame, rospy.Time(0)))

                        move_forward_trans = np.array([0, 0, .0375])
                        move_forward_t = transformations.translation_matrix(move_forward_trans)

                        hand_target_t = np.matmul(hand_t, move_forward_t) #move forward in hand frame
                        
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                        hand_target_pose = helpers.g_to_pose(hand_target_t)

                        # Might have to edit this . . . 
                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move forward, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan motion of right arm to block, anything else to skip: ") == "q":

                        tool_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", tool_frame, rospy.Time(0)))

                        tool_to_claw_target_trans = np.array([0, 0, -0.065])
                        tool_to_claw_target_rot = np.array([1, 0, 0, 0])
                        tool_to_claw_target_t = helpers.vec_to_g(tool_to_claw_target_trans, tool_to_claw_target_rot)

                        #base to claw
                        claw_target_t = np.matmul(tool_t, tool_to_claw_target_t)

                        frame_pub.publish(helpers.g_to_tf(claw_target_t))

                        claw_to_right_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(claw_frame, right_hand_frame, rospy.Time(0)))

                        right_hand_target_t = np.matmul(claw_target_t, claw_to_right_hand_t)     

                        frame_pub.publish(helpers.g_to_tf(right_hand_target_t))

                        right_hand_target_pose = helpers.g_to_pose(right_hand_target_t)

                        #right hand
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
