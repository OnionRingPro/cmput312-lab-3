#!/usr/bin/env python3
import math
from client.helpers.kinematics import inverse_kinematics
from client.helpers.movements import move_on_trajectory, calibrate_zero, move_to_position
from client.helpers.trajectories import arc_points

# n = 100
# r = 0.1

# cx = 0
# cy = 0.1

# def point(t):
#     x = r*math.cos(2*math.pi*t/n) + cx
#     y = r*math.sin(2*math.pi*t/n) + cy
#     return (x, y)

def main():
    points = [(15/100, 10/100), (10/100, 18/100), (-10/100, 15/100), (-15/100, 10/100)]
    trajectory = arc_points(points)
    # check all points are valid
    for x, y in trajectory:
        inverse_kinematics(x, y)
    calibrate_zero()

    # move to the initail point
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
