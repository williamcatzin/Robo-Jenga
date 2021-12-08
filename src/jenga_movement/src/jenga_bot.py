# fill this in with a class that has all of the services, subs, and pubs neccesarry to do the movements

class Jenga_Bot:

    BLOCK_HEIGHT = .015
    BLOCK_WIDTH = .025
    BLOCK_LENGHT = .075

    ### PRIVATE ###

    def get_rough_align_frame(self):
        """ Return rough align frame (g) """
        pass

    def get_stacking_frame(self):
        """ Return stacking frame (g), the location we start the stacking process at """
        pass

    def update_force_value(self):
        """ Load cell subscriber callback """
        pass

    def plan_stick_movement(self, g):
        """Take target stick frame relative to base and return plan to move there"""
        pass

    def plan_claw_movement(self, g):
        """Take target claw frame relative to base and return plan to move there"""
        pass

    ### PUBLIC ###

    def __init__(self):
        """ Setup Jenga Bot object. Init subscriber, service proxy"""
        pass

    def execute_stick_movement(self, plan):
        """ Execute plan for stick movement """
        pass

    def execute_claw_movement(self, plan):
        """ Execute plan for claw movement """
        pass

    def open_claw(self):
        """ Call service proxy to open claw """
        pass

    def close_claw(self):
        """ Call service proxy to close claw """
        pass

    # STICK MOTIONS #

    def plan_align_stick_to_tag(self):
        """ Return plan to align stick to tag """
        pass

    def plan_move_down_rows(self, num_rows):
        """ Return plan to move stick down num_rows rows """
        pass

    def plan_push(self):
        """ Plan full push motion to verify in RVIZ"""
        pass

    def execute_careful_push(self):
        """ Plan and execute careful push, checking force value along the way. Return bool for success"""
        pass

    def plan_stick_pull_back(self):
        """ Return plan to move stick to ready position at current block level """
        pass

    # CLAW MOTIONS #

    def plan_align_claw_to_stick(self):
        """ Return plan to move claw to offset from stick to get ready to grab block """
        pass

    def plan_block_grab(self):
        """ Return plan to move up to block to grab it """
        pass

    def plan_pull_pushed_block(self):
        """ Return plan to pull block out of tower """
        pass

    def plan_move_up_to_stack(self):
        """ Return plan to move up to height of stack """
        pass

    def plan_move_forward_to_stack(self):
        """ Return plan to move forward to stack """
        pass