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


# def inverse_kinematics(x, y, error=ERROR, initial_guess=INITIAL_GUESS):
#     """
#     Return an analytical and a numerical inverse-kinematics solution in degrees. 
#     Each angle is normalized to the range [-180, 180).
#     """
#     theta1, theta2 = numerical_inverse_kinematics(x, y, error, initial_guess)
#     theta1_analytical, theta2_analytical = analytical_inverse_kinematics(x, y)
#     return (theta1, theta2), (theta1_analytical, theta2_analytical)
    
def numerical_inverse_kinematics(x, y, error=ERROR, initial_guess=INITIAL_GUESS):
    """
    Return a numerical inverse-kinematics solution in degrees. 
    Each angle is normalized to the range [-180, 180).
    """
    theta1, theta2 = initial_guess

    for _ in range(MAX_INTERATION):
        # Calculate the current end-effector position
        current_x, current_y = forward_kinematics(theta1, theta2)

        # Calculate the error in position
        error_x = x - current_x
        error_y = y - current_y

        # Check if the error is within the acceptable range
        if math.hypot(error_x, error_y) < error:
            return _normalize_angle(theta1), _normalize_angle(theta2)

        # Calculate the Jacobian matrix
        J11 = -L1 * math.sin(math.radians(theta1)) - L2 * math.sin(math.radians(theta1 + theta2))
        J12 = -L2 * math.sin(math.radians(theta1 + theta2))
        J21 = L1 * math.cos(math.radians(theta1)) + L2 * math.cos(math.radians(theta1 + theta2))
        J22 = L2 * math.cos(math.radians(theta1 + theta2))

        # Calculate the determinant of the Jacobian
        det_J = J11 * J22 - J12 * J21

        if abs(det_J) < 1e-8:
            # Move slightly away from a singular configuration before applying
            # the inverse-Jacobian update.
            theta2 += 1.0
            continue

        # Calculate the inverse of the Jacobian matrix
        inv_J11 = J22 / det_J
        inv_J12 = -J12 / det_J
        inv_J21 = -J21 / det_J
        inv_J22 = J11 / det_J

        # Update joint angles using the inverse Jacobian and position errors
        delta_theta1 = math.degrees(inv_J11 * error_x + inv_J12 * error_y)
        delta_theta2 = math.degrees(inv_J21 * error_x + inv_J22 * error_y)

        # Close to a singularity the inverse Jacobian can produce an enormous
        # Newton step.  Scale both joint updates together so the direction is
        # preserved while keeping the iteration stable.
        max_step = 15.0
        largest_step = max(abs(delta_theta1), abs(delta_theta2))
        if largest_step > max_step:
            scale = max_step / largest_step
            delta_theta1 *= scale
            delta_theta2 *= scale

        theta1 += delta_theta1
        theta2 += delta_theta2
    raise RuntimeError("Numerical IK did not converge")

    

def analytical_inverse_kinematics(x, y):
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


def _angular_difference(angle1, angle2):
    """Return the smallest signed difference between two degree angles."""
    return _normalize_angle(angle1 - angle2)
