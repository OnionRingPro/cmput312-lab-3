#!/usr/bin/env python3
import math
from helpers.movements import move_on_trajectory, calibrate_zero
from helpers.trajectories import arc_points

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("start", metavar='float', nargs=2, type=float, help="2 coordinates in the order of 2 in cm")
    parser.add_argument("angle", metavar='float', nargs=1, type=float, help="angle in degrees")
    parser.add_argument("distance", metavar='float', nargs=1, type=float, help="distance in cm")
    args = parser.parse_args()

    point1 = map(lambda x: x/100, args.points)
    calibrate_zero()
    trajectory = line_points_from_angle(point1, args.angle, args.distance[0]/100)
    move_on_trajectory(trajectory)


if __name__ == "__main__":
    main()
