# fill this in with a class that has all of the services, subs, and pubs neccesarry to do the movements
import rospy
import helpers
import tf2_ros
import numpy as np
import tf2_msgs.msg
from path_planner import PathPlanner
from peripherals.srv import GripperSrv
import transformations
from std_msgs.msg import Float32

# Jenga Specs
BLOCK_HEIGHT = .015
BLOCK_WIDTH = .025
BLOCK_LENGTH = .075
BLOCK_OFFSET = .02

CLAW_ALIGN_DIST = BLOCK_LENGTH + 0.03
CLAW_GRAB_DIST = .04
CLAW_STACK_OFFSET = .05

MAX_FORCE = 1



class Jenga_Bot:
    # Frames:
    # aruco_tag_0, pokey_stick, left_hand, grabby_claw, right_hand

    ### PRIVATE ###

    def get_rough_align_frame(self):
        """ Return rough align frame (g) """
        pass

    def get_claw_ready_frame(self):
        """ Return calw ready frame (g) """
        pass

    def get_stacking_frame(self):
        """ Return stacking frame (g), the location we start the stacking process at """
        pass

    def get_tag_align_transform(self):
        """ Return the transform (g) from tag to target """
        tag_to_stick_target_trans = np.array([0, 0, BLOCK_OFFSET])
        tag_to_stick_target_rot = np.array([0.7071068, 0.7071068, 0, 0])
        tag_to_stick_target_t = helpers.vec_to_g(tag_to_stick_target_trans, tag_to_stick_target_rot)
        return tag_to_stick_target_t

    def update_force_value(self, force):
        """ Load cell subscriber callback """
        self.FORCE = force

    def plan_stick_movement(self, stick_target_t):
        """Take target stick frame relative to base and return plan to move there"""
        # Publish to tf
        self.frame_pub.publish(helpers.g_to_tf(stick_target_t, "base", "stick_target"))
        # Get Homogenous Matrix g from tf info
        stick_to_hand_t = helpers.tf_to_g(self.tfBuffer.lookup_transform(self.STICK_FRAME, self.LEFT_HAND_FRAME, rospy.Time(0)))
        # Get desired end-effector configuration matrix
        hand_target_t = np.matmul(stick_target_t, stick_to_hand_t)
        #CREATE HAND TARGET POSE     
        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
        # PUBLISH TO TF
        self.frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "left_hand_target"))
        # GENERATE AND EXECUTE PLAN
        plan = self.stick_planner.plan_to_pose(hand_target_pose, [])
        return plan

    def plan_claw_movement(self, claw_target_t):
        """Take target claw frame relative to base and return plan to move there"""
        # Publish to tf
        self.frame_pub.publish(helpers.g_to_tf(claw_target_t, "base", "claw_target"))
        # Get Homogenous Matrix g from tf info
        claw_to_hand_t = helpers.tf_to_g(self.tfBuffer.lookup_transform(self.CLAW_FRAME, self.RIGHT_HAND_FRAME, rospy.Time(0)))
        # Get desired end-effector configuration matrix
        hand_target_t = np.matmul(claw_target_t, claw_to_hand_t)
        #CREATE HAND TARGET POSE     
        hand_target_pose = helpers.g_to_pose(hand_target_t, "base")
        # PUBLISH TO TF
        self.frame_pub.publish(helpers.g_to_tf(hand_target_t, "base", "right_hand_target"))
        # GENERATE AND EXECUTE PLAN
        plan = self.claw_planner.plan_to_pose(hand_target_pose, [])
        return plan

    ### PUBLIC ###

    def __init__(self):
        """ Setup Jenga Bot object. Init subscriber, service proxy"""
        
        # Push Stick Frame
        self.STICK_FRAME = "pokey_stick"
        # Claw Frame
        self.CLAW_FRAME = "grabby_claw"
        # Left Hand Frame
        self.LEFT_HAND_FRAME = 'left_hand'
        # Right Hand Frame
        self.RIGHT_HAND_FRAME = 'right_hand'
        # Tag frame
        self.TAG_FRAME = "aruco_tag_0"

        self.stick_planner = PathPlanner("left_arm")
        # Right Hand
        self.claw_planner = PathPlanner("right_arm")
        rospy.Subscriber('push_force', Float32, self.update_force_value)

        self.tfBuffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(self.tfBuffer)

        # Publish to tf
        self.frame_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

        self.offsets_to_next_row = [4, 2, 2, 2, 2, 2, 2]
        self.rows_moved_to = 0
        self.total_row_offset = -3
        self.moving_down = False

    def execute_stick_movement(self, plan):
        """ Execute plan for stick movement """
    
        if not self.stick_planner.execute_plan(plan):
            raise Exception("Execution failed")

        if self.moving_down:
            self.moving_down = False
            self.total_row_offset -= self.offsets_to_next_row[self.rows_moved_to]
            self.rows_moved_to += 1
            

    def execute_claw_movement(self, plan):
        """ Execute plan for claw movement """

        if not self.claw_planner.execute_plan(plan):
            raise Exception("Execution failed")

    def open_claw(self):
        """ Call service proxy to open claw """

        rospy.wait_for_service('open_gripper')
        try:
            send_command = rospy.ServiceProxy('open_gripper', GripperSrv)
            response = send_command("open")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def close_claw(self):
        """ Call service proxy to close claw """

        rospy.wait_for_service('close_gripper')
        try:
            send_command = rospy.ServiceProxy('close_gripper', GripperSrv)
            response = send_command("close")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # STICK MOTIONS #

    def plan_stick_rough_align(self):
        """ Return plan to get stick camera pointed in right direction """

        return self.plan_stick_movement(self.get_rough_align_frame())

    def plan_align_stick_to_tag(self):
        """ Return plan to align stick to tag """

        # GET TAG TO STICK
        tag_to_stick_target_t = self.get_tag_align_transform()
        #FIND WHERE TAG IS (GET TAG HOMOGENOUS TRANSrospy.init_node('moveit_node')FORM)
        tag_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.TAG_FRAME, rospy.Time(0)))
        #GET STICK TARGET IN SPATIAL FRAME (FROM CURRENT AR TAG POSITION)
        stick_target_t = np.matmul(tag_t, tag_to_stick_target_t)
        
        plan = self.plan_stick_movement(stick_target_t)

        return plan

    def plan_move_stick_down_rows(self, num_rows):
        """ Return plan to move stick down num_rows rows """

        # Get trans matrix from base to left hand and convert to homogenous mattrix
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))
        # Move down the Jenga tower by 0.015 mm down the y-axis
        move_down_trans = np.array([0, 0, -1 * num_rows * BLOCK_HEIGHT])
        move_down_t = transformations.translation_matrix(move_down_trans)

        stick_target_t = np.matmul(move_down_t, stick_t) #move down in spatial frame

        plan = self.plan_stick_movement(stick_target_t)

        return plan

    def plan_move_to_next_row(self):
        self.moving_down = True
        return self.plan_move_stick_down_rows(self.offsets_to_next_row[self.rows_moved_to])

    def plan_push(self):
        """ Plan full push motion to verify in RVIZ"""

        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))

        move_forward_trans = np.array([0, 0, .5 * BLOCK_LENGTH + BLOCK_OFFSET])
        move_forward_t = transformations.translation_matrix(move_forward_trans)

        stick_target_t = np.matmul(stick_t, move_forward_t) #move forward in hand frame
        
        plan = self.plan_stick_movement(stick_target_t)

        return plan

    def execute_careful_push(self):
        """ Plan and execute careful push, checking force value along the way. Return bool for success"""
        distances = np.linspace(0, .5 * BLOCK_LENGTH + BLOCK_OFFSET, 20)
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))
        for dist in distances:

            move_forward_trans = np.array([0, 0, dist])
            move_forward_t = transformations.translation_matrix(move_forward_trans)

            stick_target_t = np.matmul(stick_t, move_forward_t) #move forward in hand frame
            
            plan = self.plan_stick_movement(stick_target_t)

            self.execute_stick_movement(plan)
            if self.FORCE > MAX_FORCE:
                return False
        return True

    def plan_stick_pull_back(self):
        """ Return plan to move stick to ready position at current block level """
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))

        # Move Backwards
        move_backwards_trans = np.array([0, 0, -1 *(.5 * BLOCK_LENGTH + BLOCK_OFFSET)])
        move_backwards_t = transformations.translation_matrix(move_backwards_trans)

        stick_target_t = np.matmul(stick_t, move_backwards_t) #move backward
        
        plan = self.plan_stick_movement(stick_target_t)

        return plan

    # CLAW MOTIONS #

    def plan_claw_ready_position(self):
        """ Return plan to move claw to ready position """

        return self.plan_claw_movement(self.get_claw_ready_frame())

    def plan_align_claw_to_stick(self):
        """ Return plan to move claw to offset from stick to get ready to grab block """
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))

        stick_to_claw_target_trans = np.array([0, 0, -1 * CLAW_ALIGN_DIST])
        stick_to_claw_target_rot = np.array([1, 0, 0, 0])
        stick_to_claw_target_t = helpers.vec_to_g(stick_to_claw_target_trans, stick_to_claw_target_rot)

        claw_target_t = np.matmul(stick_t, stick_to_claw_target_t)

        plan = self.plan_claw_movement(claw_target_t)

        return plan
        

    def plan_claw_to_block(self):
        """ Return plan to move up to block to grab it """
        # .04 +z
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.STICK_FRAME, rospy.Time(0)))

        stick_to_claw_target_trans = np.array([0, 0, -1 * CLAW_GRAB_DIST])
        stick_to_claw_target_rot = np.array([1, 0, 0, 0])
        stick_to_claw_target_t = helpers.vec_to_g(stick_to_claw_target_trans, stick_to_claw_target_rot)

        claw_target_t = np.matmul(stick_t, stick_to_claw_target_t)

        plan = self.plan_claw_movement(claw_target_t)

        return plan


    def plan_pull_pushed_block(self):
        """ Return plan to pull block out of tower """
        # -.04 -z
        stick_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.CLAW_FRAME, rospy.Time(0)))

        stick_to_claw_target_trans = np.array([0, 0, -1 *(.5 * BLOCK_LENGTH - BLOCK_OFFSET)])
        stick_to_claw_target_rot = np.array([1, 0, 0, 0])
        stick_to_claw_target_t = helpers.vec_to_g(stick_to_claw_target_trans, stick_to_claw_target_rot)

        claw_target_t = np.matmul(stick_t, stick_to_claw_target_t)

        plan = self.plan_claw_movement(claw_target_t)

        return plan


    def plan_move_claw_up_dist(self, dist):
        """ Return plan to move stick down num_rows rows """

        # Get trans matrix from base to left hand and convert to homogenous mattrix
        claw_t = helpers.tf_to_g(self.tfBuffer.lookup_transform("base", self.CLAW_FRAME, rospy.Time(0)))
        # Move down the Jenga tower by 0.015 mm down the y-axis
        move_up_trans = np.array([0, 0, dist])
        move_up_t = transformations.translation_matrix(move_up_trans)

        claw_target_t = np.matmul(move_up_t, claw_t) #move down in spatial frame

        plan = self.plan_claw_movement(claw_target_t)

        return plan    

    def plan_move_up_to_stack(self):
        """ Return plan to move up to height of stack """
        return self.plan_move_claw_up_dist(self.total_row_offset * BLOCK_HEIGHT + CLAW_STACK_OFFSET)

    def plan_move_forward_to_stack(self):
        """ Return plan to move forward to stack """
        