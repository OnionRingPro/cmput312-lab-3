"""Kinematics helpers for the robot arm."""

import math
from helpers.config import ERROR, L1, L2, MAX_INTERATION, INITIAL_GUESS

def forward_kinematics(theta1, theta2):
    """Return the end-effector position: numerical (x, y)."""
    theta1_radians = math.radians(theta1)
    theta12_radians = math.radians(theta1 + theta2)

    x = L1 * math.cos(theta1_radians) + L2 * math.cos(theta12_radians)
    y = L1 * math.sin(theta1_radians) + L2 * math.sin(theta12_radians)

    return x, y

def inverse_kinematics(x, y):
    """Return an analytical inverse-kinematics solution in degrees. 
    Each angle is normalized to the range [-180, 180).
    """
    # Calculate the distance from the origin to the point (x, y)
    r = math.sqrt(x**2 + y**2)

    # Check if the point is reachable
    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError("The point is unreachable.")

    # Calculate the angle for theta2 using the law of cosines
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.degrees(math.acos(cos_theta2))

    # Calculate the angle for theta1
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * math.sin(math.radians(theta2))
    theta1 = math.degrees(math.atan2(y, x) - math.atan2(k2, k1)) # change to + if elbow down

    return _normalize_angle(theta1), _normalize_angle(theta2)

def _normalize_angle(angle):
    """Normalize an angle in degrees to the range [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0

