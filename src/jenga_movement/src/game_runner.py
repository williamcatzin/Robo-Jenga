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
            # get to reliable starting position that can see tower
            pass
        elif state == 'ALIGN_STICK':
            # align stick to tower
            pass
        elif state == 'NEXT_BLOCK':
            # move to next block ready position
            pass
        elif state == 'ATTEMPT_PUSH':
            # careful push
            pass
        elif state == 'PUSH_FAIL':
            # handle case of block too hard to push
            pass
        elif state == 'ALIGN_CLAW':
            # align claw to pushed block (offset ready position)
            pass
        elif state == 'GRAB_BLOCK':
            # open claw, move claw forward, close claw
            pass
        elif state == 'REMOVE_BLOCK':
            # back up claw
            pass
        elif state == 'MOVE_TO_STACK':
            


if __name__ == '__main__':
    main(sys.argv)