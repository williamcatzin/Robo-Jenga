#!/usr/bin/env python

from os import stat
import sys
import rospy
from jenga_bot import Jenga_Bot
from enum import Enum

class State(Enum):
    START = auto()
    SETUP = auto()
    CALIBRATE = auto()
    ROUGH_ALIGN = auto()
    ALIGN_STICK = auto()
    CLAW_READY = auto()
    NEXT_BLOCK = auto()
    ATTEMPT_PUSH = auto()
    PUSH_ABORT = auto()
    ALIGN_CLAW = auto()
    RETRACT_STICK = auto()
    GRAB_BLOCK = auto()
    REMOVE_BOCK = auto()
    MOVE_TO_STACK = auto()
    PLACE_BOCK = auto()
    FIX_BLOCK = auto()
    SHUTDOWN = auto()
    EXIT = auto()



# fill this in with a state machine that creates a jenga_bot object and calls its methods according to user input

def main(args):
    rospy.init_node('game_runner', anonymous=True)
    jenga_bot = Jenga_Bot()
    
    state = State.START
    while not rospy.is_shutdown():
        if state == State.START:
            # enter state machine
            state = State.ROUGH_ALIGN #for now
        elif state == State.SETUP:
            # do any setup neccessary
            pass
        elif state == State.CALIBRATE:
            # calibrate loadcell 0 and 1
            pass
        elif state == State.ROUGH_ALIGN:
            # get to reliable starting position that can see tower (we may want to increase speed here)
            pass
        elif state == State.ALIGN_STICK:
            # align stick to tower, query user for if align is successful, if successful, save position of tower for later use, else retry
            pass
        elif state == State.CLAW_READY:
            # move claw to ready position
            pass
        elif state == State.NEXT_BLOCK:
            # move to next block ready position
            pass
        elif state == State.ATTEMPT_PUSH:
            # careful push
            pass
        elif state == State.PUSH_ABORT:
            # handle case of block too hard to push
            pass
        elif state == State.ALIGN_CLAW:
            # align claw to pushed block (offset ready position)
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_align_claw_to_stick()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                pass
            state = State.RETRACT_STICK
        elif state == State.RETRACT_STICK:
            # pull stick out of tower
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_stick_pull_back()
            except:
                pass
            raw_input("Press <Enter> to execute stick path: ")
            try:
                jenga_bot.execute_stick_movement(plan)
            except:
                pass
            state = State.GRAB_BLOCK
        elif state == State.GRAB_BLOCK:
            # open claw, move claw forward, close claw
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_block_grab()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                pass
            state = State.REMOVE_BOCK
        elif state == State.REMOVE_BOCK:
            # back up claw (we may want to increase speed here)
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_pull_pushed_block()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                pass
            state = State.REMOVE_BOCK
        elif state == State.MOVE_TO_STACK:
            # move claw up to top of tower
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_move_up_to_stack()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                pass
            state = State.PLACE_BOCK
        elif state == State.PLACE_BOCK:
            # move to proper offset, open gripper
            raw_input("Press <Enter> to plan claw path: ")
            try:
                jenga_bot.plan_move_forward_to_stack()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                pass
            
            #skip fixing block for now 
            r = raw_input("Try another block? (y/n): ")
            if r == "y":
                state = State.CLAW_READY
            else:
                state = State.SHUTDOWN

        elif state == State.FIX_BLOCK:
            # move behind block, close gripper, push block forward
            pass
        elif state == State.SHUTDOWN:
            # wave goodbye
            state = State.EXIT
        elif state == State.EXIT:
            # exit program
            break
        else:
            print("Invalid state: \"{}\". Moving to EXIT.".format(str(state)))
            state = 'EXIT'
    
    # main returns




if __name__ == '__main__':
    main(sys.argv)