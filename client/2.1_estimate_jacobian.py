#!/usr/bin/python
import helpers.movements as mv
import client

def estimate_jacobian(delta_theta_1: float, delta_theta_2: float):
    """
        Estimate the initial Jacobian matrix with a small change in the motor angles in the camera space.
        Both angle measurements are in degrees
        Used for part 2 of the lab
    """
    init_robot_angle1, init_robot_angle2 = mv.get_joint_angles()
    init_camera_pos_u, init_camera_pos_v, _ = client.request_end_effector_pos()
    print("Initial robot position: (theta1=" + str(init_robot_angle1) + ", theta2=" + str(init_robot_angle2) + ")")
    print("Initial camera position: (u=" + str(init_camera_pos_u) + ", v=" + str(init_camera_pos_v) + ")")

    mv.move_to_joint_angles(init_robot_angle1 + delta_theta_1, init_robot_angle2)

    int_robot_angle1, int_robot_angle2 = mv.get_joint_angles()
    int_camera_pos_u, int_camera_pos_v, _ = client.request_end_effector_pos()
    print("Intermediate robot position: (theta1=" + str(int_robot_angle1) + ", theta2=" + str(int_robot_angle2) + ")")
    print("Intermediate camera position: (u=" + str(int_camera_pos_u) + ", v=" + str(int_camera_pos_v) + ")")

    measured_delta_theta1_1 = int_robot_angle1 - init_robot_angle1
    measured_delta_theta2_1 = int_robot_angle2 - init_robot_angle2
    delta_u_1 = int_camera_pos_u - init_camera_pos_u
    delta_v_1 = int_camera_pos_v - init_camera_pos_v

    mv.move_to_joint_angles(init_robot_angle1, init_robot_angle2 + delta_theta_2)

    final_robot_angle1, final_robot_angle2 = mv.get_joint_angles()
    final_camera_pos_u, final_camera_pos_v, _ = client.request_end_effector_pos()
    print("Final robot position: (theta1=" + str(final_robot_angle1) + ", theta2=" + str(final_robot_angle2) + ")")
    print("Final camera position: (u=" + str(final_camera_pos_u) + ", v=" + str(final_camera_pos_v) + ")")

    measured_delta_theta1_2 = final_robot_angle1 - int_robot_angle1
    measured_delta_theta2_2 = final_robot_angle2 - int_robot_angle2
    delta_u_2 = final_camera_pos_u - int_camera_pos_u
    delta_v_2 = final_camera_pos_v - int_camera_pos_v

    


def main():
    client.setup_client()
    mv.calibrate_zero()
    estimate_jacobian(10, 10)


if __name__ == "__main__":
    main()