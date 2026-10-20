#!/usr/bin/python
import client.helpers.movements as mv
import conn.client as client

def estimate_jacobian(delta_theta_1: float, delta_theta_2: float):
    """
        Estimate the initial Jacobian matrix with a small change in the motor angles in the camera space.
        Both angle measurements are in degrees
        Used for part 2 of the lab
    """
    mv.calibrate_zero()


def main():
    client.setup_client()
    estimate_jacobian(10, 10)
    init_pos = mv.get_joint_angles()
    init_camera_pos = client.get_end_effector_pos()

    print(init_camera_pos)

if __name__ == "__main__":
    main()