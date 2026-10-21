#!/usr/bin/env python3
import math
import argparse
from client.helpers.movements import move_on_trajectory, calibrate_zero
from client.helpers.trajectories import line_points_from_angle

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("start", metavar='float', nargs=2, type=float, help="2 coordinates in the order of 2 in cm")
    parser.add_argument("angle", metavar='float', nargs=1, type=float, help="angle in degrees")
    parser.add_argument("distance", metavar='float', nargs=1, type=float, help="distance in cm")
    args = parser.parse_args()

    calibrate_zero()
    trajectory = line_points_from_angle((args.start[0]/100, args.start[1]/100), args.angle[0], args.distance[0]/100)
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
