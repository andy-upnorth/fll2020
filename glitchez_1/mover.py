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

# This sort of fast, so we might change if it is hard to steer.
ROBOT_STRAIGHT_SPEED = 250

# Set same every time - not too fast
ROBOT_TURN_RATE = 80


# Set the drive speed at 100 millimeters per second.
LINE_DRIVE_SPEED = 120


# Calculate the light threshold. Choose values based on your measurements.
BLACK = 30
WHITE = 99
line_threshold = (BLACK + WHITE) / 2




# copied from example
#
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
# PROPORTIONAL_GAIN = 3.0 # 1.2
PROPORTIONAL_GAIN_NORMAL = 1.2
PROPORTIONAL_GAIN_BIG_LEFT_TURN = 3.0



# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(common.left_motor, common.right_motor, wheel_diameter=55.5, axle_track=165)


def set_defaults():
    # This is fast, so we might lower the max later.
    robot.stop()

    robot.settings(straight_speed=ROBOT_STRAIGHT_SPEED, turn_rate=ROBOT_TURN_RATE)

    # We tried a few acceleration values.  2 is crazy slow...  
    #   400 seems to work.  the original made the robot jump when starting
    robot.distance_control.limits(acceleration=500)


# ok set them
set_defaults()


# Adjust the line follow and keep going
#   smooth turn is normal
#   for big left turn use PROPORTIONAL_GAIN_BIG_LEFT_TURN
def check_n_turn(turn_multiply = PROPORTIONAL_GAIN_NORMAL, drive_speed = LINE_DRIVE_SPEED):
    # Calculate the deviation from the threshold.
    deviation = common.line_sensor.reflection() - line_threshold
    
    # Calculate the turn rate.
    turn_rate = turn_multiply * deviation

    # Set the drive base speed and turn rate.
    robot.drive(drive_speed, turn_rate)

# Adjust the line follow and keep going
#   smooth turn is normal
#   for big left turn use PROPORTIONAL_GAIN_BIG_LEFT_TURN
def check_n_turn_left_sensor(turn_multiply = PROPORTIONAL_GAIN_NORMAL, drive_speed = LINE_DRIVE_SPEED):
    # Calculate the deviation from the threshold.
    #deviation = 65 - common.color_sensor.reflection()
    deviation = 45 - common.color_sensor.reflection()
    
    # Calculate the turn rate.
    turn_rate = turn_multiply * deviation

    # Set the drive base speed and turn rate.
    robot.drive(drive_speed, turn_rate)



'''
Driving to lines
--------------------
'''

wait_sensor_timer = StopWatch()


# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def wait_for_color_on_right_line_sensor(the_color):
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


# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def wait_for_color(the_color):
    while True:
        common.ev3.light.on(Color.YELLOW)
        if common.color_sensor.color() != the_color:
            continue # this jumps back to the start of the loop

        # Sensor found a glimmer of the_color, so now
        # start a timer and make sure that the same color is
        # visibile for a while
        wait_sensor_timer.reset()
        while common.color_sensor.color() == the_color:
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
        if common.color_sensor.color() == the_color:
            continue # this jumps back to the start of the loop

        # Sensor found at least a small spot without the_color, so now
        # start a timer and make sure that the we don't get back onto
        # the_color too soon
        wait_sensor_timer.reset()
        while common.color_sensor.color() != the_color:
            if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM: # enough time already
                return
            common.ev3.light.on(Color.RED)
            continue
        
        # otherwise go through the loop again


def drive_to_start():
    
    common.color_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(160, 0)

    # look for white 
    wait_until_not_color(Color.WHITE)

    robot.stop()


def drive_to_white_black_white_right_line_sensor(speed = 110):
    
    common.line_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(speed, 0)

    # look for white 
    wait_for_color_on_right_line_sensor(Color.WHITE)

    robot.drive(speed, 0)

    # look for black
    wait_for_color_on_right_line_sensor(Color.BLACK)
    #ev3.light.on(Color.GREEN)

    # look for white
    wait_for_color_on_right_line_sensor(Color.WHITE)
    common.ev3.light.on(Color.RED)
    robot.stop()
    


def drive_to_white_black_white_left_color_sensor(speed = 110):
    
    common.color_sensor.color() # switch to color mode

    # Begin driving forward
    #   -1 turns a little to left to fix drift
    robot.drive(speed, -0.8)

    # look for white 
    wait_for_color(Color.WHITE)

    robot.drive(speed, 0)

    # look for black
    wait_for_color(Color.BLACK)
    #ev3.light.on(Color.GREEN)

    # look for white
    wait_for_color(Color.WHITE)
    common.ev3.light.on(Color.RED)
    robot.stop()
    



def drive_to_white_on_right_line_sensor(speed):
    
    common.line_sensor.color() # switch to color mode

    # Begin driving - could be forward or backward
    robot.drive(speed, 0)

    # look for white 
    wait_for_color_on_right_line_sensor(Color.WHITE)

    robot.stop()


def drive_to_white(speed):
    
    common.color_sensor.color() # switch to color mode

    # Begin driving - could be forward or backward
    robot.drive(speed, 0)

    # look for white 
    wait_for_color(Color.WHITE)

    robot.stop()


def drive_to_green(speed):

    common.color_sensor.color() # switch to color mode

    # Begin driving - could be forward or backward
    robot.drive(speed, 0)

    # look for white
    wait_for_color(Color.GREEN)

    robot.stop()


def drive_to_line(speed = LINE_DRIVE_SPEED):

    # switch to reflection mode
    common.line_sensor.reflection()

    # drive straight ahead until it gets dark
    while common.line_sensor.reflection() > line_threshold:
        robot.drive(speed=speed, turn_rate=0)

    robot.stop()



def turn_ccw_until_right_sensor_white():

    common.color_sensor.color() # switch to color mode

    robot.drive(speed=-20, turn_rate=-40)

    # look for white
    wait_for_color(Color.WHITE)

    robot.stop()


'''
Line following
--------------------
'''

# line following copied from example
#
def follow_until_treadmill():
    common.treadmill_motor.set_dc_settings(30, 0)
    common.treadmill_motor.run(-300)
    common.line_sensor.reflection()
    wait(10)

    # Follow line until arm motor stalls.
    while (common.treadmill_motor.stalled() == False):
        check_n_turn()

        # You can wait for a short time or do other things in this loop.
        wait(10)



# line following copied from example
#
def wait_until_treadmill_stall():
    common.treadmill_motor.set_dc_settings(30, 0)
    common.treadmill_motor.run(-300)
    common.line_sensor.reflection()
    wait(10)

    # Follow line until arm motor stalls.
    while (common.treadmill_motor.stalled() == False):
        # You can wait for a short time or do other things in this loop.
        wait(10)



def follow_distance(how_far, drive_speed = LINE_DRIVE_SPEED):
    # reset distance to 0
    robot.reset()

    # ready for line follow
    common.line_sensor.reflection()

    # Follow line ...............
    while (robot.distance() < how_far):
        check_n_turn(drive_speed=drive_speed)

        # You can wait for a short time or do other things in this loop.
        wait(10)

    robot.stop()


def follow_distance_left_sensor(how_far, drive_speed = LINE_DRIVE_SPEED):
    # reset distance to 0
    robot.reset()

    # ready for line follow
    common.color_sensor.reflection()

    # Follow line ...............
    while (robot.distance() < how_far):
        check_n_turn_left_sensor(drive_speed=drive_speed)

        # You can wait for a short time or do other things in this loop.
        wait(10)

    robot.stop()


# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def follow_until_color(the_color, turn_multiply = PROPORTIONAL_GAIN_NORMAL, drive_speed = LINE_DRIVE_SPEED):
    while True:
        common.ev3.light.on(Color.YELLOW)

        check_n_turn(turn_multiply, drive_speed)

        if common.color_sensor.color() != the_color:
            continue # this jumps back to the start of the loop

        # Sensor found a glimmer of the_color, so now
        # start a timer and make sure that the same color is
        # visibile for a while
        wait_sensor_timer.reset()
        while common.color_sensor.color() == the_color:
            if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM: # enough time already
                return
            common.ev3.light.on(Color.RED)
            check_n_turn(turn_multiply, drive_speed)
            continue
        
        # Fell out of the loop because the color changed.
        # But maybe it was big enough anyway
        if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM/2: # This is long enough
            return
    
        # otherwise go through the loop again


def follow_someting(how_many, turn_multiply = PROPORTIONAL_GAIN_NORMAL):
   
    common.line_sensor.reflection() # for line following
    common.color_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(130, 0)

    for x in range (how_many):
        follow_until_color(Color.WHITE, turn_multiply=turn_multiply)
        follow_until_color(Color.BLACK, turn_multiply=turn_multiply)
        follow_until_color(Color.WHITE, turn_multiply=turn_multiply)

    robot.stop()


def follow_on_right_until_white_black_white_on_left():

    common.line_sensor.reflection()

    follow_until_color(Color.GREEN, drive_speed=100)
    follow_until_color(Color.WHITE, drive_speed=180)
    follow_until_color(Color.BLACK, drive_speed=180)

    robot.stop()


def drive_to_start_with_follow():
    
    common.line_sensor.color() # switch to color mode
    common.color_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(80, 0)

    wait_for_color_on_right_line_sensor(Color.BLACK)

    common.line_sensor.reflection() # for line following

    # if this doesn't work then make: follow_until_not_color(Color.WHITE)
    follow_until_color(Color.GREEN)
    #wait_until_not_color(Color.WHITE)

    robot.stop()


def play_minecraft():
    ev3.speaker.say("Yay 5 Mil coins")
