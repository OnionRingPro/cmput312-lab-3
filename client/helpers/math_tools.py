import math 

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

