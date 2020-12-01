#!/usr/bin/env pybricks-micropython

"""
Based on Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------
"""

import common
import mover
from mover import robot  # Make it easy to access 'robot without a prefix'

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button
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



