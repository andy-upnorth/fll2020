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

# This is fast, so we might lower the max later.

robot.settings(straight_speed=ROBOT_STRAIGHT_SPEED)


# We tried a few acceleration values.  2 is crazy slow...  
#   400 seems to work.  the original made the robot jump when starting
robot.distance_control.limits(acceleration=500)


# Adjust the line follow and keep going
#   smooth turn is normal
#   for big left turn use PROPORTIONAL_GAIN_BIG_LEFT_TURN
def check_n_turn(turn_multiply = PROPORTIONAL_GAIN_NORMAL):
    # Calculate the deviation from the threshold.
    deviation = common.line_sensor.reflection() - line_threshold
    
    # Calculate the turn rate.
    turn_rate = turn_multiply * deviation

    # Set the drive base speed and turn rate.
    robot.drive(LINE_DRIVE_SPEED, turn_rate)




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
    robot.drive(speed, 0)

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

# line following copied from example
#
def follow_until_treadmill():
    common.treadmill_motor.set_dc_settings(30, 0)
    common.treadmill_motor.run(-300)
    common.line_sensor.reflection()

    # Follow line until arm motor stalls.
    while (common.treadmill_motor.stalled() == False):
        check_n_turn()

        # You can wait for a short time or do other things in this loop.
        wait(10)


def follow_distance(how_far):
    # reset distance to 0
    robot.reset()

    # ready for line follow
    common.line_sensor.reflection()

    # Follow line ...............
    while (robot.distance() < how_far):
        check_n_turn()

        # You can wait for a short time or do other things in this loop.
        wait(10)






# This function will return after the color sensor sees
# the_color for at least COLOR_WAIT_MINIMUM milliseconds
def follow_until_color(the_color, turn_multiply):
    while True:
        common.ev3.light.on(Color.YELLOW)

        check_n_turn(turn_multiply)

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
            check_n_turn(turn_multiply)
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
        follow_until_color(Color.WHITE, turn_multiply)
        follow_until_color(Color.BLACK, turn_multiply)
        follow_until_color(Color.WHITE, turn_multiply)

    robot.stop()


