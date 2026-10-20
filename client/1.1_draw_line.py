#!/usr/bin/env python3
import argparse
from helpers.movements import move_on_trajectory, move_to_joint_angles, calibrate_zero
from helpers.trajectories import line_points

def main():

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("points", metavar='float', nargs=4, type=float, help="4 coordinates in the order of x1 y1 x2 y2 in cm")
    args = parser.parse_args()

    point1 = map(lambda x: x/100, args.points[:2])
    point2 = map(lambda x: x/100, args.points[2:])

    calibrate_zero()
    # move_to_joint_angles(0, 0)

    trajectory = line_points(point1, point2)
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
