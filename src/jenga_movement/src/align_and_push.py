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
from std_msgs.msg import Float32

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

import transformations
from peripherals.srv import GripperSrv

import tf2_ros

import tf2_msgs.msg
import geometry_msgs.msg

# Force threshold
MAX_FORCE = 2.0

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

    def LoadCellCallBack(force):
        pokey_stick.set_force(force.data)


    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                #============================================
                # GET TAG TO STICK
                tag_to_stick_target_trans = np.array([0, 0, 0])
                tag_to_stick_target_rot = np.array([0.7071068, 0.7071068, 0, 0])
                tag_to_stick_target_t = helpers.vec_to_g(tag_to_stick_target_trans, tag_to_stick_target_rot)

                #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
                tag_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", tag_frame, rospy.Time(0)))

                #GET STICK TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
                stick_target_t = np.matmul(tag_t, tag_to_stick_target_t)
                #============================================
                # PUBLISH TO TF
                
                frame_pub.publish(helpers.g_to_tf(stick_target_t, "base", stick_frame))
                #============================================

                stick_to_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(stick_frame, left_hand_frame, rospy.Time(0)))

                hand_target_t = np.matmul(stick_target_t, stick_to_hand_t)

                #CREATE HAND TARGET POSE     
                hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                #============================================
                # PUBLISH TO TF

                frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                #============================================

                if raw_input("Press q to calculate path to align to tag, anything else to skip: ") == "q":

                    #============================================
                    # GET TAG TO STICK
                    tag_to_stick_target_trans = np.array([0, 0, 0])
                    tag_to_stick_target_rot = np.array([0.7071068, 0.7071068, 0, 0])
                    tag_to_stick_target_t = helpers.vec_to_g(tag_to_stick_target_trans, tag_to_stick_target_rot)

                    #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
                    tag_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", tag_frame, rospy.Time(0)))

                    #GET STICK TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
                    stick_target_t = np.matmul(tag_t, tag_to_stick_target_t)
                    #============================================
                    # PUBLISH TO TF
                    
                    frame_pub.publish(helpers.g_to_tf(stick_target_t, "base", stick_frame))
                    #============================================

                    stick_to_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(stick_frame, left_hand_frame, rospy.Time(0)))

                    hand_target_t = np.matmul(stick_target_t, stick_to_hand_t)

                    #CREATE HAND TARGET POSE     
                    hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                    #============================================
                    # PUBLISH TO TF

                    frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                    #============================================

                    # GENERATE AND EXECUTE PLAN
                    plan = stick_planner.plan_to_pose(hand_target_pose, [])

                    print(plan)

                    if raw_input("Press q to execute plan, anything else to skip: ") == "q":
                        if not stick_planner.execute_plan(plan):
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
                        rospy.Subscriber('push_force', Float32, LoadCellCallBack)
                        pokey_stick = PokeyStick()
                        push_distance = 0.375
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", left_hand_frame, rospy.Time(0)))
                        distances = np.linspace(0, push_distance, 10)
                        for dist in distances:
                            move_forward_trans = np.array([0, 0, dist])
                            move_forward_t = transformations.translation_matrix(move_forward_trans)

                            hand_target_t = np.matmul(hand_t, move_forward_t) #move forward in hand frame
                            
                            frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                            hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                            plan = stick_planner.plan_to_pose(hand_target_pose, [])
                            FORCE = pokey_stick.get_force()
                            print("Good Force: ", FORCE)
                            if FORCE > 0.0:
                                print("Bad Force: ", FORCE)
                                # Abort Pushing Forward
                                stick_planner.shutdown()
                                break
                        for dist in distances:
                            # Move Backwards
                            move_backwards_trans = np.array([0, -1*dist, 0])
                            move_backwards_t = transformations.translation_matrix(move_backwards_trans)

                            hand_target_t = np.matmul(move_backwards_t, hand_t) #move backward
                            
                            frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))

                            hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                            plan = stick_planner.plan_to_pose(hand_target_pose, [])
                    
        
                        if raw_input("Press q to move forward, anything else to skip: ") == "q":
                            if not stick_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan motion of right arm to block, anything else to skip: ") == "q":

                        stick_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", claw_frame, rospy.Time(0)))

                        stick_to_claw_target_trans = np.array([0, 0, -0.065])
                        stick_to_claw_target_rot = np.array([1, 0, 0, 0])
                        stick_to_claw_target_t = helpers.vec_to_g(stick_to_claw_target_trans, stick_to_claw_target_rot)

                        claw_target_t = np.matmul(stick_t, stick_to_claw_target_t)

                        frame_pub.publish(helpers.g_to_tf(claw_target_t, "base", claw_frame))

                        claw_to_right_hand_t = helpers.tf_to_g(tfBuffer.lookup_transform(claw_frame, right_hand_frame, rospy.Time(0)))

                        right_hand_target_t = np.matmul(claw_target_t, claw_to_right_hand_t)     

                        frame_pub.publish(helpers.g_to_tf(right_hand_target_t, "base", right_hand_frame))

                        right_hand_target_pose = helpers.g_to_pose(right_hand_target_t, "base")

                        plan = claw_planner.plan_to_pose(right_hand_target_pose, [])


                        
                        if raw_input("Press q to move right hand, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                raise Exception("Execution failed")
                            else:
                                rospy.wait_for_service('close_gripper')
                                try:
                                    send_command = rospy.ServiceProxy('close_gripper', GripperSrv)
                                    response = send_command("P")
                                except rospy.ServiceException as e:
                                    print("Service call failed: %s"%e)

                    if raw_input("Press q to plan motion of right arm backwards, anything else to skip: ") == "q":

                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        #MOVE BACKWARDS
                        move_backwards_trans = np.array([0, -0.05, 0])
                        move_backwards_t = transformations.translation_matrix(move_backwards_trans)
                        hand_target_t = np.matmul(move_backwards_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand back, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                raise Exception("Execution failed")
                        
                    if raw_input("Press q to plan motion of right arm up tower, anything else to skip: ") == "q":
                        #MOVE UP TOWER
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        move_up_trans = np.array([0, 0, 0.1])
                        move_up_t = transformations.translation_matrix(move_up_trans)
                        hand_target_t = np.matmul(move_up_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand up tower, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                raise Exception("Execution failed")

                    if raw_input("Press q to plan motion of right arm foward, anything else to skip: ") == "q":
                        #MOVE FORWARD
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        move_forward_trans = np.array([0, 0.05, 0])
                        move_forward_t = transformations.translation_matrix(move_forward_trans)
                        hand_target_t = np.matmul(move_forward_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand forward, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                raise Exception("Execution failed")



                    


                    if raw_input("Press q to plan motion of right arm backwards, anything else to skip: ") == "q":

                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        #MOVE BACKWARDS
                        move_backwards_trans = np.array([0, -0.05, 0])
                        move_backwards_t = transformations.translation_matrix(move_backwards_trans)
                        hand_target_t = np.matmul(move_backwards_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand back, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                    raise Exception("Execution failed")
                        
                    if raw_input("Press q to plan motion of right arm up tower, anything else to skip: ") == "q":
                        #MOVE UP TOWER
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        move_up_trans = np.array([0, 0, 0.1])
                        move_up_t = transformations.translation_matrix(move_up_trans)
                        hand_target_t = np.matmul(move_up_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand up tower, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                    raise Exception("Execution failed")

                    if raw_input("Press q to plan motion of right arm foward, anything else to skip: ") == "q":
                        #MOVE FORWARD
                        hand_t = helpers.tf_to_g(tfBuffer.lookup_transform("base", right_hand_frame, rospy.Time(0)))
                        move_forward_trans = np.array([0, 0.05, 0])
                        move_forward_t = transformations.translation_matrix(move_forward_trans)
                        hand_target_t = np.matmul(move_forward_t, hand_t) #move down in spatial frame
                        frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "hand_target"))
                        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
                        plan = claw_planner.plan_to_pose(hand_target_pose, [])

                        if raw_input("Press q to move right hand forward, anything else to skip: ") == "q":
                            if not claw_planner.execute_plan(plan):
                                    raise Exception("Execution failed")



                    


            except Exception as e:
                print(e)
                traceback.print_exc()

class PokeyStick:
    def __init__(self):
        self.force = 0.0

    def set_force(self, force):
        self.force = force
    def get_force(self):
        return self.force

  

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main(sys.argv)
