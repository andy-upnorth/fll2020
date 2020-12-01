#!/usr/bin/env pybricks-micropython

"""
Main program for FLL team
-----------------------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

import common
import arm
import missions

if common.CAN_DRIVE:
    import mover
    #common.ev3.speaker.say("ready to drive")

#
# Done with setup.
#

common.ev3.light.on(Color.RED)
#common.ev3.speaker.say(" i like minecraft i want to play it now")
#beeper()


if common.CAN_DRIVE:
   
   if common.HAVE_WHEEL_ARM:
        # Quick testing
        #missions.wiggle_step()
        #missions.turn_from_white_bar_to_arch()


        # do entire step counter - and the treadmill
        missions.do_step_counter()

        missions.do_treadmill_after_steps()
        missions.drive_long_way_after_treadmill()

        # Do entire treadmill mission from start instead
        #missions.do_treadmill_from_start()

    else:
        # Do other missions that do not need the treadmill arm
        missions.do_pullup()


else:
    # do somthing with the arm
    #arm.wag_arm()
    common.ev3.speaker.say(" i like minecraft i want to play it now")

