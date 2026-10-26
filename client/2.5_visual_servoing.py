#!/usr/bin/python
from client import setup_client
import helpers.movements as mv
from helpers.math_tools import estimate_jacobian, Broyden
import math
import time
import argparse

ERROR = 5.0  # pixels
CONTROL_GAIN = 0.05 # controls how large a step to take in the direction of the target
MAX_STEP = math.radians(3.0) # maximum joint change per iteration, in radians
MAX_ITERATIONS = 100


def read_positions(client):
    positions = client.request_positions()
    while positions is None:
        print("Waiting for both markers...")
        time.sleep(0.1)
        positions = client.request_positions()
    return positions


def update_jacobian(J, previous_angles, current_angles,
                    previous_position, current_position, step_size=1):
    delta_q = [current_angles[i] - previous_angles[i] for i in range(2)]
    delta_y = [current_position[i] - previous_position[i] for i in range(2)]

    # Skip negligible joint motion to avoid dividing by almost zero.
    if delta_q[0]**2 + delta_q[1]**2 <= 1e-8:
        return J

    return Broyden(J, delta_y, delta_q, step_size)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--track", action="store_true", help="Keep following a moving target until Ctrl+C.")
    args = parser.parse_args()
    client = setup_client()
    try:
        mv.calibrate_zero()
        read_positions(client)  # Wait until the camera can see both markers.
        run_servoing(client, continuous=args.track)
    finally:
        mv.l1_motor.off(brake=True)
        mv.l2_motor.off(brake=True)
        client.s.close()


def run_servoing(client, continuous=False):
    jacobian = estimate_jacobian(client, 10, 10)
    end_effector_pos, target_pos = read_positions(client)

    iteration = 0
    while continuous or iteration <= MAX_ITERATIONS:
        error = [target_pos[i] - end_effector_pos[i] for i in range(2)]
        distance = math.hypot(*error)
        previous_angles = mv.get_joint_angles()
        print("Iteration:", iteration, "Target:", target_pos,
              "End effector:", end_effector_pos, "Error (pixels):", distance,
              "Joint angles: ", previous_angles
              )
        if distance <= ERROR:
            if not continuous:
                print("Target reached.")
                return True
            time.sleep(0.1)
            end_effector_pos, target_pos = read_positions(client)
            continue
        if not continuous and iteration == MAX_ITERATIONS:
            print("Iteration limit reached; target was not reached.")
            return False

        # delta_q = gain * inverse(J) * (target - end_effector).
        a, b = jacobian[0]
        c, d = jacobian[1]
        determinant = a * d - b * c
        if abs(determinant) < 1e-8:
            print("Jacobian is singular. Reposition the arm and try again.")
            return False
        delta_q = [CONTROL_GAIN * (d * error[0] - b * error[1]) / determinant,
                   CONTROL_GAIN * (a * error[1] - c * error[0]) / determinant]
        largest_step = max(abs(value) for value in delta_q)
        if largest_step > MAX_STEP:
            delta_q = [value * MAX_STEP / largest_step for value in delta_q]
        previous_position = end_effector_pos

        # Synchronize the movement helper with the actual encoder readings.
        mv.set_position()
        mv.move_to_joint_angles(previous_angles[0] + (delta_q[0] * 180 / math.pi),
                                previous_angles[1] + (delta_q[1] * 180 / math.pi)
        )
        end_effector_pos, target_pos = read_positions(client)
        current_angles = mv.get_joint_angles()
        previous_angles_rad = [a * math.pi / 180 for a in previous_angles]
        current_angles_rad = [a * math.pi / 180 for a in current_angles]
        jacobian = update_jacobian(jacobian, previous_angles_rad, current_angles_rad,
                                   previous_position, end_effector_pos)
        iteration += 1


if __name__ == "__main__":
    main()
