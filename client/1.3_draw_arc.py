#!/usr/bin/env python3
import math
from helpers.movements import move_on_trajectory, calibrate_zero, move_to_position
from helpers.trajectories import arc_points
from helpers.config import L1, L2

n = 10
r = 0.05

cx = 0.08
cy = 0.08

def circle(t):
    x = r*math.cos(2*math.pi*t/n) + cx
    y = r*math.sin(2*math.pi*t/n) + cy
    return (x, y)

def main():

    points = [circle(t) for t in range(n)]
    #points = [(15/100, 10/100), (10/100, 18/100), (-10/100, 15/100), (-15/100, 10/100)]
    trajectory = arc_points(points)
    # check all points are valid
    for x, y in trajectory:
        assert x**2 + y**2 - L1 - L2 < 1e-6
    calibrate_zero()

    # move to the initail point
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
