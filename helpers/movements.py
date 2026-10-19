#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, LargeMotor, SpeedDPS, SpeedPercent
from helpers.config import MAX_POS, MIN_POS, MOTOR1_RATIO, MOTOR2_RATIO
import math
import time
from helpers.kinematics import analytical_inverse_kinematics, numerical_inverse_kinematics

l1_motor = LargeMotor(OUTPUT_A)
l2_motor = LargeMotor(OUTPUT_B)
l1_motor_zero = 0
l2_motor_zero = 0
l1_motor_current = 0
l2_motor_current = 0

def calibrate_zero():
    """Calibrate the zero position of the robot arm."""
    global l1_motor_zero, l2_motor_zero, l1_motor_current, l2_motor_current
    l1_motor_current = -90
    l2_motor_current = -91
    l1_motor_zero = l1_motor.degrees - l1_motor_current
    l2_motor_zero = l2_motor.degrees - l1_motor_current - l2_motor_current*MOTOR2_RATIO

def move_to_joint_angles(theta1, theta2):
    # assert l1_motor_zero is not None and l2_motor_zero is not None, "Motors must be calibrated before moving."
    global l1_motor_current, l2_motor_current
    # motor1_reset = (-l1_motor_current) * MOTOR1_RATIO
    # motor2_reset = (-l2_motor_current) * MOTOR2_RATIO
    # motor1_target = (theta1) * MOTOR1_RATIO
    # motor2_target = (theta2) * MOTOR2_RATIO
    #
    # # Reset Motor 1 In Tandem
    # l1_motor.on_for_degrees(SpeedDPS(25), motor1_reset,  brake=False, block=False)
    # l2_motor.on_for_degrees(SpeedDPS(25), motor1_reset, brake=False, block=True)
    #
    # # Reset Motor 2
    # l2_motor.on_for_degrees(SpeedDPS(25), motor2_reset, brake=False, block=True)
    #
    # # Move to Target Motor 1 In Tandem
    # l1_motor.on_for_degrees(SpeedDPS(25), motor1_target, brake=False, block=False)
    # l2_motor.on_for_degrees(SpeedDPS(25), motor1_target, brake=False, block=True)
    #
    # # Move to Target Motor 2 In Tandem
    # l2_motor.on_for_degrees(SpeedDPS(25), motor2_target, brake=False, block=True)

    motor1_target = (theta1 - l1_motor_current) * MOTOR1_RATIO
    motor2_target = (theta2 - l2_motor_current) * MOTOR2_RATIO

    l1_motor.on_for_degrees(SpeedDPS(60), motor1_target, brake=False, block=False)
    # l2_motor.on_for_degrees(SpeedDPS(60), motor1_target, brake=False, block=True)
    l2_motor.on_for_degrees(SpeedDPS(60), motor1_target + motor2_target, brake=False, block=True)
    l1_motor.wait_while("running")

    l1_motor_current = theta1
    l2_motor_current = theta2

def get_computed_angles():
    return l1_motor_current, l2_motor_current

def set_position():
    global l1_motor_current, l2_motor_current
    theta1, theta2 = get_joint_angles()
    l1_motor_current = theta1
    l2_motor_current = theta2 

def get_joint_angles():
    """Return the current joint angles (theta1, theta2) in degrees."""
    assert l1_motor_zero is not None and l2_motor_zero is not None, "Motors must be calibrated before getting joint angles."
    theta1 = (l1_motor.degrees - l1_motor_zero) / (MOTOR1_RATIO)
    theta2 = ((l2_motor.degrees - theta1 - l2_motor_zero) / (MOTOR2_RATIO))
    return theta1, theta2


def move_to_position(x, y, numeric=True):
    """
    Move the robot arm to the specified (x, y) position.
    The angles are numerical angles calculated by the inverse kinematics function.
    """
    theta1, theta2 = (0,0)
    if numeric:
        set_position()
        theta1, theta2 = numerical_inverse_kinematics(x,y, initial_guess=(l1_motor_current, l2_motor_current))
    else:
        theta1, theta2 = analytical_inverse_kinematics(x,y)

    move_to_joint_angles(theta1, theta2)
    return theta1, theta2
