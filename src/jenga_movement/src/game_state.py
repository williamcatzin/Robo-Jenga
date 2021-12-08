#!/usr/bin/env python

from enum import Enum, auto

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
