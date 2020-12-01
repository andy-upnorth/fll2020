#!/usr/bin/env pybricks-micropython

"""
Based on Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------
"""

import common

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# Minimum millseconds to wait to make sure a color band is wide enough
COLOR_WAIT_MINIMUM = 160

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(common.left_motor, common.right_motor, wheel_diameter=55.5, axle_track=165)

# We tried a few acceleration values.  2 is crazy slow...  
#   400 seems to work.  the original made the robot jump when starting
robot.distance_control.limits(acceleration=500)


wait_sensor_timer = StopWatch()


# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def wait_for_color(the_color):
    while True:
        common.ev3.light.on(Color.YELLOW)
        if common.line_sensor.color() != the_color:
            continue # this jumps back to the start of the loop

        # Sensor found a glimmer of the_color, so now
        # start a timer and make sure that the same color is
        # visibile for a while
        wait_sensor_timer.reset()
        while common.line_sensor.color() == the_color:
            if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM: # enough time already
                return
            common.ev3.light.on(Color.RED)
            continue
        
        # Fell out of the loop because the color changed.
        # But maybe it was big enough anyway
        if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM/2: # This is long enough
            return
    
        # otherwise go through the loop again


# This function will return after the color sensor has NOT seen
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def wait_until_not_color(the_color):
    while True:
        common.ev3.light.on(Color.YELLOW)
        if common.line_sensor.color() == the_color:
            continue # this jumps back to the start of the loop

        # Sensor found at least a small spot without the_color, so now
        # start a timer and make sure that the we don't get back onto
        # the_color too soon
        wait_sensor_timer.reset()
        while common.line_sensor.color() != the_color:
            if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM: # enough time already
                return
            common.ev3.light.on(Color.RED)
            continue
        
        # otherwise go through the loop again


def drive_to_start():
    
    common.line_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(270, 0)

    # look for white 
    wait_until_not_color(Color.WHITE)

    robot.stop()


def drive_to_white_black_white():
    
    common.line_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(130, 0)

    # look for white 
    wait_for_color(Color.WHITE)

    robot.drive(90, 0)

    # look for black
    wait_for_color(Color.BLACK)
    #ev3.light.on(Color.GREEN)

    # look for white
    wait_for_color(Color.WHITE)
    common.ev3.light.on(Color.RED)
    robot.stop()
    


def drive_to_white(speed):
    
    common.line_sensor.color() # switch to color mode

    # Begin driving - could be forward or backward
    robot.drive(speed, 0)

    # look for white 
    wait_for_color(Color.WHITE)

    robot.stop()



def drive_to_second_and_turn():
    common.ev3.light.on(Color.RED)
    common.ev3.light.on(Color.GREEN)

    drive_to_start()

    # reset the distance counter
    robot.reset()

    # first 
    drive_to_white_black_white()

    # second one
    drive_to_white_black_white()


    robot.turn(-90)
    robot.straight(200)
    # now maybe follow line


