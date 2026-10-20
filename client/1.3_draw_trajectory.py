#!/usr/bin/env python3
import math
from helpers.movements import move_on_trajectory, move_to_joint_angles, calibrate_zero
from helpers.trajectories import line_points

n = 100
r = 10

cx = 0
cy = 10

def point(t):
    x = r*math.cos(2*math.pi*t/n) + cx
    y = r*math.sin(2*math.pi*t/n) + cy
    return (x, y)

def main():
    trajectory = [point(t) for t in range(0, n-1)]
    calibrate_zero()
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
