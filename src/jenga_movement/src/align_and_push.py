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
                tag_to_tool_target_t = np.matmul(transformations.quaternion_matrix(tag_to_tool_target_rot), transformations.translation_matrix(tag_to_tool_target_trans))
                # tag_to_tool_target_t = transformations.identity_matrix()

                #tool_target = tag * tag_to_target

                #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
                tag_transform = tfBuffer.lookup_transform("base", tag_frame, rospy.Time(0))
                tag_trans = np.array([tag_transform.transform.translation.x, tag_transform.transform.translation.y, tag_transform.transform.translation.z])
                tag_rot = np.array([tag_transform.transform.rotation.x, tag_transform.transform.rotation.y, tag_transform.transform.rotation.z, tag_transform.transform.rotation.w])
                tag_t = np.matmul(transformations.translation_matrix(tag_trans), transformations.quaternion_matrix(tag_rot))

                #GET TOOL TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
                tool_target_t = np.matmul(tag_t, tag_to_tool_target_t)


                #============================================

                # PUBLISH TO RVIZ
                tool_target_trans = transformations.translation_from_matrix(tool_target_t)
                tool_target_rot = transformations.quaternion_from_matrix(tool_target_t)

                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "base"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "tool_target"
                t.transform.translation.x = tool_target_trans[0]
                t.transform.translation.y = tool_target_trans[1]
                t.transform.translation.z = tool_target_trans[2]

                t.transform.rotation.x = tool_target_rot[0]
                t.transform.rotation.y = tool_target_rot[1]
                t.transform.rotation.z = tool_target_rot[2]
                t.transform.rotation.w = tool_target_rot[3]
                
                tfm = tf2_msgs.msg.TFMessage([t])
                frame_pub.publish(tfm)
                #============================================

                # (THIS IS CONSTANT) tool frame to hand frame
                #hand_target = tool_to hand * tool_target (tool_to_hand is defined in a static transform, so it should always be the same)
                tool_to_hand_transform = tfBuffer.lookup_transform(tool_frame, hand_frame, rospy.Time(0))
                tool_to_hand_trans = np.array([tool_to_hand_transform.transform.translation.x, tool_to_hand_transform.transform.translation.y, tool_to_hand_transform.transform.translation.z])
                tool_to_hand_rot = np.array([tool_to_hand_transform.transform.rotation.x, tool_to_hand_transform.transform.rotation.y, tool_to_hand_transform.transform.rotation.z, tool_to_hand_transform.transform.rotation.w])
                tool_to_hand_t = np.matmul(transformations.translation_matrix(tool_to_hand_trans), transformations.quaternion_matrix(tool_to_hand_rot))

                hand_target_t = np.matmul(tool_target_t, tool_to_hand_t)

                #CONVERT TO ROTATION QUATERNION AND T VECTOR
                hand_target_trans = transformations.translation_from_matrix(hand_target_t)
                hand_target_rot = transformations.quaternion_from_matrix(hand_target_t)
                #============================================

                #CREATE HAND TARGET POSE     
                hand_target_pose = PoseStamped()
                hand_target_pose.header.frame_id = "base"

                #x, y, and z position
                hand_target_pose.pose.position.x = hand_target_trans[0]
                hand_target_pose.pose.position.y = hand_target_trans[1]
                hand_target_pose.pose.position.z = hand_target_trans[2]

                #Orientation as a quaternion
                hand_target_pose.pose.orientation.x = hand_target_rot[0]
                hand_target_pose.pose.orientation.y = hand_target_rot[1]
                hand_target_pose.pose.orientation.z = hand_target_rot[2]
                hand_target_pose.pose.orientation.w = hand_target_rot[3]
                #============================================

                #RVIZ STUFF
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "base"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "hand_target"
                t.transform.translation.x = hand_target_trans[0]
                t.transform.translation.y = hand_target_trans[1]
                t.transform.translation.z = hand_target_trans[2]

                t.transform.rotation.x = hand_target_rot[0]
                t.transform.rotation.y = hand_target_rot[1]
                t.transform.rotation.z = hand_target_rot[2]
                t.transform.rotation.w = hand_target_rot[3]

                tfm = tf2_msgs.msg.TFMessage([t])
                frame_pub.publish(tfm)
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

                #initialize dummy variable for final pose
                final_pose = PoseStamped()
                final_trans = np.array([])
                final_trans = np.array([])

                while(True):
                    
                    if raw_input("Press q to plan movement down, anything else to skip: ") == "q":

                        hand_transform = tfBuffer.lookup_transform("base", hand_frame, rospy.Time(0))
                        hand_trans = np.array([hand_transform.transform.translation.x, hand_transform.transform.translation.y, ha0.0254
                        move_down_trans = np.array([0, 0, -.015])
                        move_down_t = transformations.translation_matrix(move_down_trans)

                        hand_target_t = np.matmul(move_down_t, hand_t) #move down in spatial frame

                        hand_target_trans = transformations.translation_from_matrix(hand_target_t)
                        hand_target_rot = transformations.quaternion_from_matrix(hand_target_t)

                        hand_target_pose = PoseStamped()
                        hand_target_pose.header.frame_id = "base"
        
                        #x, y, and z position
                        hand_target_pose.pose.position.x = hand_target_trans[0]
                        hand_target_pose.pose.position.y = hand_target_trans[1]
                        hand_target_pose.pose.position.z = hand_target_trans[2]
        
                        #Orientation as a quaternion
                        hand_target_pose.pose.orientation.x = hand_target_rot[0]
                        hand_target_pose.pose.orientation.y = hand_target_rot[1]
                        hand_target_pose.pose.orientation.z = hand_target_rot[2]
                        hand_target_pose.pose.orientation.w = hand_target_rot[3]

                        t = geometry_msgs.msg.TransformStamped()
                        t.header.frame_id = "base"
                        t.header.stamp = rospy.Time.now()
                        t.child_frame_id = "hand_target"
                        t.transform.translation.x = hand_target_trans[0]
                        t.transform.translation.y = hand_target_trans[1]
                        t.transform.translation.z = hand_target_trans[2]

                        t.transform.rotation.x = hand_target_rot[0]
                        t.transform.rotation.y = hand_target_rot[1]
                        t.transform.rotation.z = hand_target_rot[2]
                        t.transform.rotation.w = hand_target_rot[3]

                        tfm = tf2_msgs.msg.TFMessage([t])
                        frame_pub.publish(tfm)
        
                        # Might have to edit this . . . 
                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move down, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan movement forward, ahything else to skip: ") == "q":

                        hand_transform = tfBuffer.lookup_transform("base", hand_frame, rospy.Time(0))
                        hand_trans = np.array([hand_transform.transform.translation.x, hand_transform.transform.translation.y, hand_transform.transform.translation.z])
                        hand_rot = np.array([hand_transform.transform.rotation.x, hand_transform.transform.rotation.y, hand_transform.transform.rotation.z, hand_transform.transform.rotation.w])
                        hand_t = np.matmul(transformations.translation_matrix(hand_trans), transformations.quaternion_matrix(hand_rot))

                        move_forward_trans = np.array([0, 0, .0375])
                        move_forward_t = transformations.translation_matrix(move_forward_trans)

                        hand_target_t = np.matmul(hand_t, move_forward_t) #move forward in hand frame

                        hand_target_trans = transformations.translation_from_matrix(hand_target_t)
                        hand_target_rot = transformations.quaternion_from_matrix(hand_target_t)
                        

                        hand_target_pose = PoseStamped()
                        hand_target_pose.header.frame_id = "base"
        
                        #x, y, and z position
                        hand_target_pose.pose.position.x = hand_target_trans[0]
                        hand_target_pose.pose.position.y = hand_target_trans[1]
                        hand_target_pose.pose.position.z = hand_target_trans[2]
        
                        #Orientation as a quaternion
                        hand_target_pose.pose.orientation.x = hand_target_rot[0]
                        hand_target_pose.pose.orientation.y = hand_target_rot[1]
                        hand_target_pose.pose.orientation.z = hand_target_rot[2]
                        hand_target_pose.pose.orientation.w = hand_target_rot[3]

                        t = geometry_msgs.msg.TransformStamped()
                        t.header.frame_id = "base"
                        t.header.stamp = rospy.Time.now()
                        t.child_frame_id = "hand_target"
                        t.transform.translation.x = hand_target_trans[0]
                        t.transform.translation.y = hand_target_trans[1]
                        t.transform.translation.z = hand_target_trans[2]

                        t.transform.rotation.x = hand_target_rot[0]
                        t.transform.rotation.y = hand_target_rot[1]
                        t.transform.rotation.z = hand_target_rot[2]
                        t.transform.rotation.w = hand_target_rot[3]

                        tfm = tf2_msgs.msg.TFMessage([t])
                        frame_pub.publish(tfm)
        
                        # Might have to edit this . . . 
                        plan = stick_planner.plan_to_pose(hand_target_pose, [])

                        print(plan)
        
                        if raw_input("Press q to move forward, anything else to skip: ") == "q":
        
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                        if raw_input("Press q to plan motion of right arm to block, anything else to skip: ") == "q":

                            tool_transform = tfBuffer.lookup_transform("base", tool_frame, rospy.Time(0))
                            tool_trans = np.array([tool_transform.transform.translation.x, tool_transform.transform.translation.y, tool_transform.transform.translation.z])
                            tool_rot = np.array([tool_transform.transform.rotation.x, tool_transform.transform.rotation.y, tool_transform.transform.rotation.z, tool_transform.transform.rotation.w])
                            tool_t = np.matmul(transformations.translation_matrix(tool_trans), transformations.quaternion_matrix(tool_rot))

                            tool_to_claw_target_trans = np.array([0, 0, -0.065])
                            tool_to_claw_target_rot = np.array([1, 0, 0, 0])
                            tool_to_claw_target_t = np.matmul(transformations.quaternion_matrix(tool_to_claw_target_rot), transformations.translation_matrix(tool_to_claw_target_trans))

                            #base to claw
                            claw_target_t = np.matmul(tool_t, tool_to_claw_target_t)

                            claw_target_trans = transformations.translation_from_matrix(claw_target_t)
                            claw_target_rot = transformations.quaternion_from_matrix(claw_target_t) 


                            t = geometry_msgs.msg.TransformStamped()
                            t.header.frame_id = "base"
                            t.header.stamp = rospy.Time.now()
                            t.child_frame_id = "claw_target"
                            t.transform.translation.x = claw_target_trans[0]
                            t.transform.translation.y = claw_target_trans[1]
                            t.transform.translation.z = claw_target_trans[2]

                            t.transform.rotation.x = claw_target_rot[0]
                            t.transform.rotation.y = claw_target_rot[1]
                            t.transform.rotation.z = claw_target_rot[2]
                            t.transform.rotation.w = claw_target_rot[3]

                            tfm = tf2_msgs.msg.TFMessage([t])
                            frame_pub.publish(tfm)


                            claw_to_right_hand_transform = tfBuffer.lookup_transform(claw_frame, right_hand_frame, rospy.Time(0))
                            claw_to_right_hand_trans = np.array([claw_to_right_hand_transform.transform.translation.x, claw_to_right_hand_transform.transform.translation.y, claw_to_right_hand_transform.transform.translation.z])
                            claw_to_right_hand_rot = np.array([claw_to_right_hand_transform.transform.rotation.x, claw_to_right_hand_transform.transform.rotation.y, claw_to_right_hand_transform.transform.rotation.z, claw_to_right_hand_transform.transform.rotation.w])
                            claw_to_right_hand_t = np.matmul(transformations.translation_matrix(claw_to_right_hand_trans), transformations.quaternion_matrix(claw_to_right_hand_rot))

                            right_hand_target_t = np.matmul(claw_target_t, claw_to_right_hand_t)
                            right_hand_target_trans = transformations.translation_from_matrix(right_hand_target_t)
                            right_hand_target_rot = transformations.quaternion_from_matrix(right_hand_target_t)         

                            right_hand_target_pose = PoseStamped()
                            right_hand_target_pose.header.frame_id = "base"
            
                            #x, y, and z position
                            right_hand_target_pose.pose.position.x = right_hand_target_trans[0]
                            right_hand_target_pose.pose.position.y = right_hand_target_trans[1]
                            right_hand_target_pose.pose.position.z = right_hand_target_trans[2]
            
                            #Orientation as a quaternion
                            right_hand_target_pose.pose.orientation.x = right_hand_target_rot[0]
                            right_hand_target_pose.pose.orientation.y = right_hand_target_rot[1]
                            right_hand_target_pose.pose.orientation.z = right_hand_target_rot[2]
                            right_hand_target_pose.pose.orientation.w = right_hand_target_rot[3]

                            t = geometry_msgs.msg.TransformStamped()
                            t.header.frame_id = "base"
                            t.header.stamp = rospy.Time.now()
                            t.child_frame_id = "right_hand_target"
                            t.transform.translation.x = right_hand_target_trans[0]
                            t.transform.translation.y = right_hand_target_trans[1]
                            t.transform.translation.z = right_hand_target_trans[2]

                            t.transform.rotation.x = right_hand_target_rot[0]
                            t.transform.rotation.y = right_hand_target_rot[1]
                            t.transform.rotation.z = right_hand_target_rot[2]
                            t.transform.rotation.w = right_hand_target_rot[3]

                            tfm = tf2_msgs.msg.TFMessage([t])
                            frame_pub.publish(tfm)

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
