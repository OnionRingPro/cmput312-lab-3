"""Geometry helpers used by the robot arm measurement programs."""

import math

def distance_between(point1, point2):
    """Return the Euclidean distance between two (x, y) points."""
    x1, y1 = point1
    x2, y2 = point2
    return math.hypot(x2 - x1, y2 - y1)


def angle_between(vertex, point1, point2):
    """Return the smaller angle between two lines in degrees.
    cos(theta) = (v1 . v2) / (|v1| * |v2|)
    """
    vx, vy = vertex
    vector1 = (point1[0] - vx, point1[1] - vy)
    vector2 = (point2[0] - vx, point2[1] - vy)

    magnitude1 = math.hypot(vector1[0], vector1[1])
    magnitude2 = math.hypot(vector2[0], vector2[1])
    if magnitude1 == 0 or magnitude2 == 0:
        raise ValueError("Points on the lines must be different from the vertex")

    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
    cosine = dot_product / (magnitude1 * magnitude2)

    # Floating-point rounding can produce values just outside [-1, 1].
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))
