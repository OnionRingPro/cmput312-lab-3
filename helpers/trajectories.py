"""Generate Cartesian waypoints without accessing the robot motors."""

import math


def line_points(start, end, step_size=0.02):
    """Return both endpoints and line waypoints got by interpolation between two points.
    """
    if not math.isfinite(step_size) or step_size <= 0:
        raise ValueError("step_size must be finite and positive")

    x1, y1 = start
    x2, y2 = end
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        raise ValueError("Point coordinates must be finite")

    distance = math.hypot(x2 - x1, y2 - y1)
    if distance == 0:
        return [(x1, y1)]

    num_steps = max(1, math.ceil(distance / step_size))
    points = [(x1, y1)]
    for i in range(1, num_steps):
        t = i / num_steps
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        points.append((x, y))
    points.append((x2, y2))
    return points


def line_points_from_angle(start, angle_deg, distance, step_size=0.02):
    """
    Return a line defined by a start point, angle, and nonnegative distance.
    """
    if not math.isfinite(angle_deg):
        raise ValueError("angle_deg must be finite")
    if not math.isfinite(distance) or distance < 0:
        raise ValueError("distance must be finite and nonnegative")

    x, y = start
    angle_rad = math.radians(angle_deg)
    end = (
        x + distance * math.cos(angle_rad),
        y + distance * math.sin(angle_rad),
    )
    return line_points(start, end, step_size)



def arc_points(points: list[tuple[float, float]], step_size=0.02):
    pass  