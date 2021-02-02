#!/usr/bin/env pybricks-micropython

"""
Main program for FLL team
-----------------------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot

TODO:
* Consider adding an initial series of beeps to indicate mission (1 beep for mission 1, etc...)
* Several more seconds can be trimmed by increasing speeds in untuned sections.
* Remaining unreliable portions:
  * When exiting the step counter it will sometimes get confused after refinding the line
    on its way to the treadmill. We're doing a 3 point turn to drop the robot right on the line
    but it still happens. I suspect it's because we're setting the stall limit on the treadmill
    motor to a hair trigger, which is actually triggering a stall when we start spinning it.
    This problem appears to happen most during low battery charge. We tried changing it
  * The weight machine worked perfectly for a while during the middle of our sessions but we
    didn't do anything to change the logic. We only saw it working when the weight machine was
    set to "yellow". It was close enough I think it could be made pretty reliable.
  * Pullup mission (#3) sometimes executes when trying to run basketball mission (#2) because
    the gyro is detected as being unplugged even when it's plugged in. Replugging helps.
  * The final left turn in mission 3 toward the pullup bar sometimes drops the line. It only
    seems to happen when the battery is low, and might benefit from a slightly bigger "back
    up to find the line" step. (comment "back up and make final left turn to find new line")

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


def do_part_1():
    #
    # Run steps/treadmill/weight missions
    #

    # Do the missions
    missions.do_step_counter()
    missions.do_treadmill_after_steps()
    missions.drive_to_north_line_after_treadmill()
    missions.do_weights_from_north_line()

    missions.drive_home_from_weights()


def do_part_2():
    # Run boccia/basket/bench missions
    #

    #mover.drive_to_start()  # just drive until not white
    mover.drive_to_start_with_follow()   # try to follow black line

    # follow line around corner
    mover.follow_someting(1, turn_multiply = mover.PROPORTIONAL_GAIN_BIG_LEFT_TURN)

    # follow two more white-black-white
    mover.follow_someting(2)

    missions.do_boccia()
    missions.do_lift_basket()
    missions.do_smash_bench()

def do_part_3():
    #
    # Run pullup bar missions
    #

    # drive to the pullup bar and go through and back out
    missions.drive_under_pullup_bar()

    # finish the mission by doing a pullup
    missions.do_pullup()





'''
jump to any part from menu
'''


def do_menu():

  common.play_hello()
  common.ev3.speaker.set_volume(25)

  while True:
    if Button.UP in common.ev3.buttons.pressed():
      common.ev3.speaker.play_notes(['A4/4'])
      #arm.to_low_start()
      do_part_1()
      common.ev3.speaker.say("ready")

    if Button.LEFT in common.ev3.buttons.pressed():
      common.ev3.speaker.play_notes(['A4/8', 'A4/8'])
      arm.to_low_start()
      mover.set_defaults()
      do_part_2()
      common.ev3.speaker.say("ready")

    if Button.RIGHT in common.ev3.buttons.pressed():
      common.ev3.speaker.play_notes(['A4/16', 'A4/16', 'A4/16'])
      arm.to_low_start()
      mover.set_defaults()
      do_part_3()
      common.ev3.speaker.say("ready")

    if Button.CENTER in common.ev3.buttons.pressed():
      common.ev3.speaker.say("ouch")

    common.ev3.light.on(Color.GREEN)
    wait(120)
    common.ev3.light.on(Color.YELLOW)




'''
can still turn off menu if it breaks
'''
USE_MENU=True

if USE_MENU:
  do_menu()
else:

  if common.CAN_DRIVE:
    
      if common.HAVE_WHEEL_ARM:
        do_part_1()       

      elif common.HAVE_GYRO:
        do_part_2()

      else:
        do_part_3()     

  else:
      # do somthing with the arm
      common.ev3.speaker.say(" i like minecraft i want to play it now")
