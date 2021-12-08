#!/usr/bin/env python

from os import stat
import sys
import rospy
from jenga_bot import Jenga_Bot
from game_state import State


def main(args):
    rospy.init_node('game_runner', anonymous=True)
    jenga_bot = Jenga_Bot()
    
    state = State.START
    while not rospy.is_shutdown():
        if state == State.START:
            print("START\n")
            # enter state machine
            state = State.ALIGN_STICK #for now
        elif state == State.SETUP:
            print("SETUP\n")
            # do any setup neccessary
            pass
        elif state == State.CALIBRATE:
            print("CALIBRATE\n")
            # calibrate loadcell 0 and 1
            pass
        elif state == State.ROUGH_ALIGN:
            print("ROUGH_ALIGN\n")
            # get to reliable starting position that can see tower (we may want to increase speed here)
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_stick_rough_align()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute stick path: ")
            try:
                jenga_bot.execute_stick_movement(plan)
            except:
                print("Execution failed")
                continue
            state = State.ALIGN_STICK
        elif state == State.ALIGN_STICK:
            print("ALIGN_STICK\n")
            # align stick to tower, query user for if align is successful, if successful, save position of tower for later use, else retry
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_align_stick_to_tag()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute stick path: ")
            try:
                jenga_bot.execute_stick_movement(plan)
            except:
                print("Execution failed")
                continue
            # TODO: Accept alignment?
            # TODO: Save tag frame
            state = State.CLAW_READY
        elif state == State.CLAW_READY:
            print("CLAW_READY\n")
            # move claw to ready position
            state = State.NEXT_BLOCK
        elif state == State.NEXT_BLOCK:
            print("NEXT_BLOCK")
            # move to next block ready position
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_move_to_next_row()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute stick path: ")
            try:
                jenga_bot.execute_stick_movement(plan)
            except:
                print("Execution failed")
                continue
            state = State.ATTEMPT_PUSH
        elif state == State.ATTEMPT_PUSH:
            print("ATTEMPT_PUSH\n")
            # careful push
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_push()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute stick path: ")
            try:
                # jenga_bot.execute_stick_movement(plan)
                if not jenga_bot.execute_careful_push():
                    state = State.PUSH_ABORT
                    continue
            except:
                print("Execution failed")
                continue
            state = State.ALIGN_CLAW
        elif state == State.PUSH_ABORT:
            print("PUSH_ABORT\n")
            # handle case of block too hard to push
            raw_input("Press <Enter> to plan stick path: ")
            try:
                plan = jenga_bot.plan_stick_pull_back()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute stick path: ")
            try:
                jenga_bot.execute_stick_movement(plan)
            except:
                print("Execution failed")
                continue
            state = State.NEXT_BLOCK
        elif state == State.ALIGN_CLAW:
            print("ALIGN_CLAW\n")
            # align claw to pushed block (offset ready position)
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_align_claw_to_stick()
            except:
                print("Planning failed")
                continue
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                print("Execution failed")
                continue
            state = State.GRAB_BLOCK
        elif state == State.GRAB_BLOCK:
            print("GRAB_BLOCK\n")
            # open claw, move claw forward, close claw
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_block_grab()
            except:
                pass
            raw_input("Press <Enter> to execute claw path: ")
            # jenga_bot.open_claw()
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                print("Execution failed")
                continue
            # jenga_bot.close_claw()
            state = State.REMOVE_BLOCK
        elif state == State.REMOVE_BLOCK:
            print("REMOVE_BLOCK\n")
            # back up claw (we may want to increase speed here)
            raw_input("Press <Enter> to plan claw path: ")
            try:
                plan = jenga_bot.plan_pull_pushed_block()
            except:
                print("Execution failed")
                continue
            raw_input("Press <Enter> to execute claw path: ")
            try:
                jenga_bot.execute_claw_movement(plan)
            except:
                print("Execution failed")
                continue
            state = State.MOVE_TO_STACK
        elif state == State.MOVE_TO_STACK:
            print("MOVE_TO_STACK\n")
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
                print("Execution failed")
                continue
            state = State.PLACE_BLOCK
        elif state == State.PLACE_BLOCK:
            print("PLACE_BLOCK\n")
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
                print("Execution failed")
                continue
            # jenga_bot.open_claw()
            #skip fixing block for now 
            r = raw_input("Try another block? (y/n): ")
            if r == "y":
                state = State.RETRACT_STICK
            else:
                state = State.SHUTDOWN
        elif state == State.RETRACT_STICK:
            print("RETRACT_STICK\n")
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
                print("Execution failed")
                continue
            state = State.CLAW_READY
        elif state == State.FIX_BLOCK:
            print("FIX_BLOCK\n")
            # move behind block, close gripper, push block forward
            pass
        elif state == State.SHUTDOWN:
            print("SHUTDOWN\n")
            # wave goodbye
            state = State.EXIT
        elif state == State.EXIT:
            print("EXIT\n")
            # exit program
            break
        else:
            print("Invalid state: \"{}\". Moving to EXIT.".format(str(state)))
            state = 'EXIT'
    
    # main returns




if __name__ == '__main__':
    main(sys.argv)