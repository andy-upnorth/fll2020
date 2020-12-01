#!/usr/bin/env pybricks-micropython

"""
Based on Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------
"""

import common
import mover
from mover import robot  # Make it easy to access 'robot without a prefix'
from common import ev3

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button, Stop, Direction
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase


def wiggle_step():

    
    for x in range (13):
        robot.straight(-20)
        robot.turn(-0.6)

        # slow wiggle works better
        robot.stop()
        robot.settings(straight_speed=55)
        robot.straight(37)
    
        # faster again
        robot.stop()
        robot.settings(straight_speed = mover.ROBOT_STRAIGHT_SPEED)


    # back up until we get to white bar
    robot.straight(-100)
    mover.drive_to_white(-100)



def turn_from_white_bar_to_arch():

    # this starts already on white.  move ahead
    robot.straight(20) 

    robot.turn(-20)
    robot.straight(110)
    robot.turn(-20)
    #robot.straight(50)
    robot.straight(120)

    #robot.drive(140, -65)
    robot.reset()
    #robot.drive(100, -60)

    robot.drive(80, 0)
    mover.wait_for_color(Color.BLACK)
    robot.straight(40)
    robot.turn(20)
    robot.stop()


def do_step_counter():

    mover.drive_to_start()

    mover.drive_to_white_black_white()

    wiggle_step()

    turn_from_white_bar_to_arch()



'''
Treadmill
----------
'''

def climb_and_spin_treadmill():

    # Reset arm motor defaults.
    common.treadmill_motor.stop()
    robot.stop()
    common.treadmill_motor.set_dc_settings(100, 0)

    # Climb onto treadmill using all 3 wheels.
    CLIMB_SPEED=100
    CLIMB_ANGLE=120
    common.treadmill_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, False)
    common.left_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, False)
    common.right_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, True)
    ev3.speaker.beep()

    # Spin the treadmill by a specified angle.
    TREADMILL_SPEED=550
    TREADMILL_ANGLE=1700
    common.treadmill_motor.run_angle(TREADMILL_SPEED, TREADMILL_ANGLE, Stop.BRAKE)

    # Climb off treadmill using all 3 wheels.
    common.treadmill_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)
    common.left_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)
    common.right_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)

    for i in range(2):
        ev3.speaker.beep(duration=500)
        wait(450)


def do_treadmill_from_start():

    # Drive forward up to 30 centimeters until the robot finds the line.
    while common.line_sensor.reflection() > mover.line_threshold:
        robot.drive(mover.LINE_DRIVE_SPEED, 0)
    ev3.speaker.beep()

    mover.follow_until_treadmill()

    climb_and_spin_treadmill()

def do_treadmill_after_steps():

    # Continue on line to and do treadmill
    mover.follow_until_treadmill()
    climb_and_spin_treadmill()


'''
Weight machine after treadmill
'''

def do_weights_after_treadmill():
    robot.turn(-45)
    robot.straight(100)

    # more to come.
    # does not work with this
    #climb_and_spin_treadmill()




'''
Pull up
'''

def do_pullup():

    # todo: drive here and line up

    # todo: get arm back and ready

    # Pull up
    common.arm.run_angle(-500, 250, Stop.BRAKE)

    ev3.beep()

    # todo: wait a long time.



