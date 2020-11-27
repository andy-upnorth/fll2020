#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
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

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S2)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

arm = Motor(Port.D)

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=165)

# We tried a few acceleration values.  2 is crazy slow...  
#   400 seems to work.  the original made the robot jump when starting
robot.distance_control.limits(acceleration=500)


ev3.speaker.set_speech_options(language='en-uk-north', voice='croak')
#ev3.speaker.set_speech_options(language='en-sc', voice='f1')

#ev3.speaker.say(" i like minecraft i want to play it now")

def beeper():
    ev3.speaker.play_notes(['C4/4', 'C4/4', 'G4/4', 'G4/4'])

# Play a sound to tell us when we are ready to start moving
#ev3.speaker.beep()

wait_timer = StopWatch()


def wait_for_color(the_color):
    while True:
        ev3.light.on(Color.YELLOW)
        if line_sensor.color() != the_color:
            continue
        wait_timer.reset()
        while line_sensor.color() == the_color:
            if wait_timer.time() > 120: # enough time already
                return
            ev3.light.on(Color.RED)
            continue
        
        if wait_timer.time() > 80: # This is long enough
            return
    
        # otherwise go through the loop again



def wait_until_not_color(the_color):
    while True:
        ev3.light.on(Color.YELLOW)
        if line_sensor.color() == the_color:
            continue
        wait_timer.reset()
        while line_sensor.color() != the_color:
            if wait_timer.time() > 120: # enough time already
                return
            ev3.light.on(Color.RED)
            continue
        
        if wait_timer.time() > 80: # This is long enough
            return
    
        # otherwise go through the loop again



def drive_to_start():
    ev3.speaker.say("Giddy up")
    line_sensor.color() # switch to color mode

    # Begin driving forward
    robot.drive(600, 0)

    # look for white 
    wait_until_not_color(Color.WHITE)

    robot.stop()



def drive_to_white_black_white():
    ev3.speaker.say("drive to white, black, white")
    line_sensor.color() # switch to color mode

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
    ev3.light.on(Color.RED)
    robot.stop()
    
    ev3.speaker.say("arrived")


ev3.light.on(Color.RED)
# beeper()

# move the arm down before starting
arm.run_until_stalled(500)
arm.run_angle(100, -20)



ev3.light.on(Color.GREEN)

#drive_to_start()

# reset the distance counter
robot.reset()

# first 
drive_to_white_black_white()

# second one
drive_to_white_black_white()


robot.turn(-90)
robot.straight(200)
# now maybe follow line


# return 
#robot.straight(-200)
#robot.straight(-1 * robot.distance())

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.
#while True:

    





    # # Begin driving forward at 200 millimeters per second.
    # robot.drive(200, 0)

    # # Wait until an obstacle is detected. This is done by repeatedly
    # # doing nothing (waiting for 10 milliseconds) while the measured
    # # distance is still greater than 300 mm.
    # while obstacle_sensor.distance() > 300:
    #     wait(10)

    # robot.stop()
    # beeper()

    # # Drive backward for 300 millimeters.
    # robot.straight(-300)

    # # Turn around by 120 degrees
    # robot.turn(120)
