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

# Set the drive speed at 100 millimeters per second.
LINE_DRIVE_SPEED = 90


# Calculate the light threshold. Choose values based on your measurements.
BLACK = 11
WHITE = 88
line_threshold = (BLACK + WHITE) / 2



# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(common.left_motor, common.right_motor, wheel_diameter=55.5, axle_track=165)

# This is fast, so we might lower the max later.

robot.settings(straight_speed=ROBOT_STRAIGHT_SPEED)


# We tried a few acceleration values.  2 is crazy slow...  
#   400 seems to work.  the original made the robot jump when starting
robot.distance_control.limits(acceleration=500)


# Adjust the line follow and keep going
def check_n_turn():
    # Calculate the deviation from the threshold.
    deviation = common.line_sensor.reflection() - line_threshold
    
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(LINE_DRIVE_SPEED, turn_rate)




'''
Driving to lines
--------------------
'''

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


GYRO_NORTH = -84
GYRO_WEST = -170

def turn_north(turn_rate):
    robot.stop()
    robot.drive(0, turn_rate)
    while(common.gyro.angle() > GYRO_NORTH):
        continue

    robot.stop()
    current_angle = common.gyro.angle()
    print("Angle is ." + str(current_angle))

def turn_west(turn_rate):
    robot.stop()
    robot.drive(0, turn_rate)
    while(common.gyro.angle() > GYRO_WEST):
        continue

    robot.stop()
    current_angle = common.gyro.angle()
    print("Angle is ." + str(current_angle))

'''
Line following
--------------------
'''

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 3.0 # 1.2


def follow_until_treadmill():
    common.treadmill_motor.set_dc_settings(30, 0)
    common.treadmill_motor.run(-300)

    # Follow line until arm motor stalls.
    while (common.treadmill_motor.stalled() == False):
        # Calculate the deviation from the threshold.
        deviation = common.line_sensor.reflection() - line_threshold
        
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(LINE_DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)


def follow_distance(how_far):
    robot.reset()

    # Follow line ...............
    while (robot.distance() < how_far):
        # Calculate the deviation from the threshold.
        deviation = common.line_sensor.reflection() - line_threshold
        
        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(LINE_DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)






# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def follow_until_color(the_color):
    while True:
        common.ev3.light.on(Color.YELLOW)

        check_n_turn()

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
            check_n_turn()
            continue
        
        # Fell out of the loop because the color changed.
        # But maybe it was big enough anyway
        if wait_sensor_timer.time() > COLOR_WAIT_MINIMUM/2: # This is long enough
            return
    
        # otherwise go through the loop again


def follow_someting(how_many):
   
    common.line_sensor.reflection() # for line following
    common.color_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(130, 0)

    for x in range (how_many):
        follow_until_color(Color.WHITE)
        follow_until_color(Color.BLACK)
        follow_until_color(Color.WHITE)

    robot.stop()


