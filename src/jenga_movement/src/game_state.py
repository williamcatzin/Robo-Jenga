#!/usr/bin/env python

from enum import Enum

class State(Enum):
    START = 1
    SETUP = 2
    CALIBRATE = 3
    ROUGH_ALIGN = 4
    ALIGN_STICK = 5
    CLAW_READY = 6
    NEXT_BLOCK = 7
    ATTEMPT_PUSH = 8
    PUSH_ABORT = 9
    ALIGN_CLAW = 10
    RETRACT_STICK = 11
    GRAB_BLOCK = 12
    REMOVE_BLOCK = 13
    MOVE_TO_STACK = 14
    PLACE_BLOCK = 15
    FIX_BLOCK = 16
    SHUTDOWN = 17
    EXIT = 18
