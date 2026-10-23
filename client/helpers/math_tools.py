import math 
import helpers.movements as mv

def CubicSpline(xs, ys):
    """
    A simple implementation of Catmull-Rom cubic spline interpolation for a set of points. 
    This function returns a callable that can be used to evaluate the spline at any point.
    """
    n = len(xs)
    if n < 2:
        raise ValueError("At least two points are required for cubic spline interpolation.")

    # Cubic Splines are defined by the functions
    # h_i(t) = (2t^3 - 3t^2 + 1)*y[i]
    #      + (t^3 - 2t^2 + t)*m[1]*(x[i+1] - x[i])
    #      + (-2t^3 + 3t^2)*y[i+1]
    #      + (t^3 - t^2)*m[2]*(x[i+1] - x[i])
    # where t is in [0, 1] and in general t = (x - x[i]) / (x[i+1] - x[i])

    # m1 and m2 are the slopes at each point, which we will compute using finite differences    
    m1 = [0] * (n-1)
    m2 = [0] * (n-1)

    for i in range(1, n-1):
        m1[i] = (ys[i+1] - ys[i-1]) / (xs[i+1] - xs[i-1])
        if i == n-2:
            m2[i] = 0
        else:
            m2[i] = (ys[i+2] - ys[i]) / (xs[i+2] - xs[i])

    def spline(x):
        if x < xs[0] or x > xs[-1]:
            raise ValueError("x is out of bounds for the spline interpolation.")

        # Find the right interval for x
        idx = 0
        for i in range(n-1):
            if xs[i] <= x <= xs[i+1]:
                idx = i
                break

        # Normalize t to [0, 1]
        t = (x - xs[idx]) / (xs[idx+1] - xs[idx])

        # Compute the spline value using the cubic spline formula
        h = xs[idx+1] - xs[idx]
        return (
            (2*t**3 - 3*t**2 + 1)*ys[idx] + 
            (t**3 - 2*t**2 + t)*m1[idx]*h + 
            (-2*t**3 + 3*t**2)*ys[idx+1] + 
            (t**3 - t**2)*m2[idx]*h
        )
    return spline



if __name__ == "__main__":
    def plot_points(points):
        import matplotlib.pyplot as plt
        # Example usage of the CubicSpline function
        n = len(points)
        xs = [x for x,y in points]
        ys = [y for x,y in points]
        ts = [i/(n-1) for i in range(n)]
        spline_x = CubicSpline(ts, xs)
        spline_y = CubicSpline(ts, ys)

        N = 50
        t_vals = [i/N for i in range(N)]
        
        print(max(ts))
        print(max(t_vals))
        x_vals = [spline_x(t) for t in t_vals] 
        y_vals = [spline_y(t) for t in t_vals] 
        plt.figure()
        plt.plot(xs, ys, 'ro', label='Control Points')
        plt.plot(x_vals, y_vals, 'b-', label='Cubic Spline')
        plt.legend()
        plt.title('Cubic Spline Interpolation')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.grid()
        plt.show()

    points = [(15/100, 10/100), (10/100, 18/100), (-10/100, 15/100), (-15/100, 10/100)]
    plot_points(points)

def estimate_jacobian(client, delta_theta_1: float, delta_theta_2: float):
    """
        Estimate the initial Jacobian matrix with a small change in the motor angles in the camera space.
        Both angle measurements are in degrees
        Used for part 2 of the lab
    """
    init_robot_angle1, init_robot_angle2 = mv.get_joint_angles()
    init_camera_pos_u, init_camera_pos_v, _ = client.request_end_effector_pos()
    print("Initial robot position: (theta1=" + str(init_robot_angle1) + ", theta2=" + str(init_robot_angle2) + ")")
    print("Initial camera position: (u=" + str(init_camera_pos_u) + ", v=" + str(init_camera_pos_v) + ")")
    print("Expected intermediate motor angles: (theta1=" + str(init_robot_angle1 + delta_theta_1) + ", theta2=" + str(init_robot_angle2) + ")")

    mv.move_to_joint_angles(init_robot_angle1 + delta_theta_1, init_robot_angle2)

    print("Expeted: " + str(mv.get_computed_angles()))

    int_robot_angle1, int_robot_angle2 = mv.get_joint_angles()
    int_camera_pos_u, int_camera_pos_v, _ = client.request_end_effector_pos()
    print("Intermediate robot position: (theta1=" + str(int_robot_angle1) + ", theta2=" + str(int_robot_angle2) + ")")
    print("Intermediate camera position: (u=" + str(int_camera_pos_u) + ", v=" + str(int_camera_pos_v) + ")")

    measured_delta_theta1_1 = int_robot_angle1 - init_robot_angle1
    measured_delta_theta2_1 = int_robot_angle2 - init_robot_angle2
    delta_u_1 = int_camera_pos_u - init_camera_pos_u
    delta_v_1 = int_camera_pos_v - init_camera_pos_v

    mv.move_to_joint_angles(int_robot_angle1, int_robot_angle2 + delta_theta_2)

    final_robot_angle1, final_robot_angle2 = mv.get_joint_angles()
    final_camera_pos_u, final_camera_pos_v, _ = client.request_end_effector_pos()
    print("Final robot position: (theta1=" + str(final_robot_angle1) + ", theta2=" + str(final_robot_angle2) + ")")
    print("Final camera position: (u=" + str(final_camera_pos_u) + ", v=" + str(final_camera_pos_v) + ")")

    measured_delta_theta1_2 = final_robot_angle1 - int_robot_angle1
    measured_delta_theta2_2 = final_robot_angle2 - int_robot_angle2
    delta_u_2 = final_camera_pos_u - int_camera_pos_u
    delta_v_2 = final_camera_pos_v - int_camera_pos_v

    jacobian = [[delta_u_1 / measured_delta_theta1_1, delta_u_2 / measured_delta_theta2_2],
                [delta_v_1 / measured_delta_theta1_1, delta_v_2 / measured_delta_theta2_2]]
    print("Estimated Jacobian: " + str(jacobian))
    return jacobian


def Broyden(J, delta_y, delta_q, step_size=0.01):
    """
        Broyden's method for updating the Jacobian matrix
        Used for part 2 of the lab
        Input:
            J: current Jacobian matrix
            delta_y: observed image-space displacement
            delta_q: change in joint angles
            step_size: step size for the update
        Return: J_(k+1)
    """
    denominator  = delta_q[0]**2 + delta_q[1]**2
    if denominator == 0:
        raise ValueError("delta_q cannot be zero vector for Broyden's update.")
    J_kq_k = [J[0][0] * delta_q[0] + J[0][1] * delta_q[1], J[1][0] * delta_q[0] + J[1][1] * delta_q[1]]
    temp = [delta_y[0] - J_kq_k[0], delta_y[1] - J_kq_k[1]]
    numeriter = [[temp[0] * delta_q[0], temp[0] * delta_q[1]], [temp[1] * delta_q[0], temp[1] * delta_q[1]]]
    new_J = [[],[]]
    for i in range(2):
        for j in range(2):
            new_J[i].append(J[i][j] + step_size * numeriter[i][j] / denominator)
    return new_J
