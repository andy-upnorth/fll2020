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
#left_motor = Motor(Port.B)
#right_motor = Motor(Port.C)

arm = Motor(Port.D)



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
    #robot.drive(600, 0)

    # look for white 
    wait_until_not_color(Color.WHITE)

    #robot.stop()

#
# This function simply wags the arm around until the
# ultrasonic distance sensor doesn't detect
# anything within 100cm  (1000mm)
#
def wag_arm():

    ev3.light.on(Color.GREEN)

    arm.run_angle(100, -180)

    while obstacle_sensor.distance() < 1000:

        while obstacle_sensor.distance() < 300:
            ev3.light.on(Color.YELLOW)
            arm.run_angle(100, 120)
            arm.run_angle(100, -120)
            wait(10)

        ev3.light.on(Color.GREEN)
        arm.run_angle(100, 60)
        arm.run_angle(100, -60)

    ev3.light.on(Color.RED)
    arm.run_angle(100, 150)



ev3.light.on(Color.RED)
# beeper()

# move the arm down before starting
arm.run_until_stalled(400)
arm.run_angle(100, -20)

# do somthing with the arm
wag_arm()


ev3.speaker.say("the end")
