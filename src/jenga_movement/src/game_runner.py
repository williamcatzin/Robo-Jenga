#!/usr/bin/env python

import sys
import rospy
from jenga_bot import Jenga_Bot

# fill this in with a state machine that creates a jenga_bot object and calls its methods according to user input

def main(args):
    rospy.init_node('game_runner', anonymous=True)
    jenga_bot = Jenga_Bot()
    
    state = "START"
    while not rospy.is_shutdown():
        if state == 'START':
            # enter state machine
            pass
        elif state == 'SETUP':
            # do any setup neccessary
            pass
        elif state == 'CALIBRATE':
            # calibrate loadcell 0 and 1
            pass
        elif state == 'ROUGH_ALIGN':
            # get to reliable starting position that can see tower (we may want to increase speed here)
            pass
        elif state == 'ALIGN_STICK':
            # align stick to tower, query user for if align is successful, if successful, save position of tower for later use, else retry
            pass
        elif state == 'CLAW_READY':
            # move claw to ready position
            pass
        elif state == 'NEXT_BLOCK':
            # move to next block ready position
            pass
        elif state == 'ATTEMPT_PUSH':
            # careful push
            pass
        elif state == 'PUSH_ABORT':
            # handle case of block too hard to push
            pass
        elif state == 'ALIGN_CLAW':
            # align claw to pushed block (offset ready position)
            plan = jenga_bot.plan_align_claw_to_stick()
            jenga_bot.execute_claw_movement(plan)
            pass
        elif state == 'RETRACT_STICK':
            # pull stick out of tower
            pass
        elif state == 'GRAB_BLOCK':
            # open claw, move claw forward, close claw
            pass
        elif state == 'REMOVE_BLOCK':
            # back up claw (we may want to increase speed here)
            pass
        elif state == 'MOVE_TO_STACK':
            # move claw up to top of tower
            pass
        elif state == 'PLACE_BLOCK':
            # move to proper offset, open gripper
            pass
        elif state == 'FIX_BLOCK':
            # move behind block, close gripper, push block forward
            pass
        elif state == 'SHUTDOWN':
            # wave goodbye
            pass
        elif state == 'EXIT':
            # exit program
            pass
        else:
            print("Invalid state: \"{}\". Moving to EXIT.".format(str(state)))
            state = 'EXIT'
    
    # main returns




if __name__ == '__main__':
    main(sys.argv)