#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, LargeMotor, SpeedDPS, SpeedPercent
from helpers.config import MAX_POS, MIN_POS, MOTOR1_RATIO, MOTOR2_RATIO
import math
import time
from helpers.kinematics import numerical_inverse_kinematics

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

def move_joint_1(theta1):
    global l1_motor_current
    motor1_target = (theta1 - l1_motor_current) * MOTOR1_RATIO
    l1_motor.on_for_degrees(SpeedDPS(60), motor1_target, brake=False, block=False)
    l1_motor.wait_while("running")
    l1_motor_current += motor1_target / MOTOR1_RATIO

def move_joint_2(theta2):
    global l2_motor_current
    motor2_target = (theta2 - l2_motor_current) * MOTOR2_RATIO
    l2_motor.on_for_degrees(SpeedDPS(60), motor2_target, brake=False, block=False)
    l2_motor.wait_while("running")
    l2_motor_current += motor2_target / MOTOR2_RATIO

def move_to_joint_angles(theta1, theta2, blocking=True):
    global l1_motor_current, l2_motor_current

    motor1_target = (theta1 - l1_motor_current) * MOTOR1_RATIO
    motor2_target = (theta2 - l2_motor_current) * MOTOR2_RATIO

    l1_motor.on_for_degrees(SpeedDPS(60), motor1_target, brake=False, block=False)
    l2_motor.on_for_degrees(SpeedDPS(60), motor1_target + motor2_target, brake=False, block=True)
    if blocking:
        l1_motor.wait_while("running")

    l1_motor_current += motor1_target / MOTOR1_RATIO
    l2_motor_current += motor2_target / MOTOR2_RATIO

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


def move_to_position(x, y):
    """
    Move the robot arm to the specified (x, y) position.
    The angles are numerical angles calculated by the inverse kinematics function.
    """
    theta1, theta2 = numerical_inverse_kinematics(x,y)

    move_to_joint_angles(theta1, theta2)
    return theta1, theta2

def move_on_trajectory(trajectory):
    """
    Move the robot arm along a trajectory defined by a list of (x, y) points.
    The angles are numerical angles calculated by the inverse kinematics function.
    """
    # for points in zip(trajectory[:-1], trajectory[1:]):
    #     move_betwen_points(points[0], points[1])
    for x, y in trajectory:
        move_to_position(x,y)
        set_position()

