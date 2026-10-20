#!/usr/bin/env python3
import math
from helpers.movements import move_on_trajectory, calibrate_zero

n = 100
r = 0.1

cx = 0
cy = 0.1

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
