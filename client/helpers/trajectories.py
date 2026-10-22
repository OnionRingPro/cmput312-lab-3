"""Generate Cartesian waypoints without accessing the robot motors."""

import math
import helpers.math_tools
# from scipy.interpolate import CubicSpline


def line_points(start, end, step_size=0.01):
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

        

def arc_points(points: list, step_size=0.005):
    if not math.isfinite(step_size) or step_size <= 0:
        raise ValueError("step_size must be finite and positive")

    points = [tuple(point) for point in points]
    if len(points) < 2:
        raise ValueError("At least two points are required")
    for point in points:
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError("Each point must contain two finite coordinates")

    for start, end in zip(points, points[1:]):
        if start == end:
            raise ValueError("Consecutive points must be different")

    if len(points) == 2:
        return line_points(points[0], points[1], step_size)

    # Two splines: one for t-x, and one for t-y. (t is just a index)
    point_numbers = list(range(len(points)))
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]

    # With scipy
    # spline_x = CubicSpline(point_numbers, xs, bc_type="natural")
    # spline_y = CubicSpline(point_numbers, ys, bc_type="natural")

    spline_x = CubicSpline(point_numbers, xs)
    spline_y = CubicSpline(point_numbers, ys)

    results = [points[0]]
    for i in range(len(points) - 1):
        distance = math.hypot(
            xs[i + 1] - xs[i], ys[i + 1] - ys[i]
        )

        # distance between two points --> split to small chunks with length==step_size to get num_steps
        # use t as the independent variable for the spline functions to get x(t) and y(t)
        num_steps = max(10, math.ceil(distance / step_size))
        for j in range(1, num_steps + 1):
            t = i + j / num_steps
            next_point = (float(spline_x(t)), float(spline_y(t)))
            if j == num_steps:
                next_point = points[i + 1]  # Keep the exact input endpoint.

            small_steps = line_points(results[-1], next_point, step_size)
            results.extend(small_steps[1:])
    return results
