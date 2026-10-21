#!/usr/bin/env python3
import argparse
from client.helpers.movements import move_on_trajectory, calibrate_zero, move_to_position
from client.helpers.trajectories import line_points

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("points", metavar='float', nargs=4, type=float, help="4 coordinates in the order of x1 y1 x2 y2 in cm")
    args = parser.parse_args()

    point1 = [s/100 for s in args.points[:2]]
    point2 = [s/100 for s in args.points[2:]]

    calibrate_zero()

    move_to_position(point1[0], point1[1])
    trajectory = line_points(point1, point2)
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
