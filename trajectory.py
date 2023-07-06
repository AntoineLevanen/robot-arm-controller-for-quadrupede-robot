import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline as Spline
from numpy import sin, cos

class sinTrajectory:

    def __init__(self, q0, omega, amplitude):
        self.q0 = q0.copy()
        self.q = q0.copy()
        self.dq = np.zeros((len(self.q0)-1, ))
        self.d2q = np.zeros((len(self.q0)-1, ))
        self.amplitude = amplitude
        self.omega = omega # frequency

    def pos(self, idx, t):
        self.q.flat[:] = self.q0
        self.q.flat[idx] += self.amplitude * np.sin(self.omega * t)
        return self.q

    def vel(self, idx, t):
        self.dq.flat[idx] = self.omega * self.amplitude * np.cos(self.omega * t)
        return self.dq

    def acc(self, idx, t):
        self.d2q.flat[idx] = -self.omega**2 * self.amplitude * np.sin(self.omega * t)
        return self.d2q


class Trajectory:

    def __init__(self, control_point, ):
        self.control_point = control_point
        self.spline = Spline(self.control_point[0], self.control_point[1])
        self.first_time = True

    def get_point_3d(self, i):
        
        
        return self.spline(i)

class CircleTrajectory:

    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.d2x = []
        self.d2y = []
        self.d2z = [] 

    def circleTrajectoryXY(self, x_center, y_center, z_center, raduis, omega):
        
        self.x = []
        self.y = []
        self.z = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.d2x = []
        self.d2y = []
        self.d2z = []
        from numpy import deg2rad as d2r
        for i in range(360):
            self.x.append(x_center + raduis * sin(omega * d2r(i)))
            self.y.append(y_center + raduis * cos(omega * d2r(i)))
            self.z.append(z_center)

            self.dx.append(omega * raduis * cos(omega * d2r(i)))
            self.dy.append(- omega * raduis * sin(omega * d2r(i)))
            self.dz.append(0)

            self.d2x.append(- omega**2 * raduis * sin(omega * d2r(i)))
            self.d2y.append(- omega**2 * raduis * cos(omega * d2r(i)))
            self.d2z.append(0)

    def circleTrajectoryYZ(self, x_center, y_center, z_center, raduis, omega):
        
        self.x = []
        self.y = []
        self.z = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.d2x = []
        self.d2y = []
        self.d2z = []

        for i in range(360):
            self.x.append(x_center)
            self.y.append(y_center + raduis * sin(i))
            self.z.append(z_center + raduis * cos(i))

            self.dx.append(0)
            self.dy.append(omega * raduis * cos(i))
            self.dz.append(-omega * raduis * sin(i))

            self.d2x.append(0)
            self.d2y.append(-omega**2 * raduis * sin(i))
            self.d2z.append(-omega**2 * raduis * cos(i))

    def getPoint(self, i):
        if i > len(self.x):
            print("Index out of range!")
            sys.exit(0)

        return [[self.x[i], self.y[i], self.z[i]], [self.dx[i], self.dy[i], self.dz[i]], [self.d2x[i], self.d2y[i], self.d2z[i]]]



def mainSinTrajectory():
    qdes = sinTrajectory(np.array([0, 0, 0]), omega=20, amplitude=0.3)
    dt = 0.01
    idx = 0

    err1x = []
    err2x = []
    err3x = []

    for i in range(400):
        t = i * dt
        err1x.append(qdes.pos(idx, t)[idx])
        err2x.append(qdes.vel(idx, t)[idx])
        err3x.append(qdes.acc(idx, t)[idx])


    plt.subplot(3, 1, 1)
    plt.plot(err1x)

    plt.subplot(3, 1, 2)
    plt.plot(err2x)

    plt.subplot(3, 1, 3)
    plt.plot(err3x)

    plt.show()

def mainTrajectory():
    control_points = [[0, 1, 2, 3], [0, 2, 0, -0.5]]
    traj = Trajectory(control_points)
    

    result_list = []

    for i in range(10):
        a = traj.get_point_3d(i)
        print(a)
        result_list.append(a)


    plt.plot(result_list[:][0], result_list[:][1])
    plt.show()

def mainCircleTrajectory():
    my_trajectory = CircleTrajectory()
    my_trajectory.circleTrajectoryXY(0.4, -0.05, 0.2, 0.14, 8)

    dt = 0.01
    idx = 0

    err1x = []
    err1y = []
    err2x = []
    err2y = []
    err3x = []
    err3y = []


    for i in range(360):
        t = i * dt
        err1x.append(my_trajectory.getPoint(i)[0][0])
        err1y.append(my_trajectory.getPoint(i)[0][1])

        err2x.append(my_trajectory.getPoint(i)[1][0])
        err2y.append(my_trajectory.getPoint(i)[1][1])
        
        err3x.append(my_trajectory.getPoint(i)[2][0])
        err3y.append(my_trajectory.getPoint(i)[2][1])

    plt.subplot(1, 3, 1)
    plt.plot(err1x, err1y)

    plt.subplot(1, 3, 2)
    plt.plot(err2x, err2y)

    plt.subplot(1, 3, 3)
    plt.plot(err3x, err3y)

    plt.show()


if __name__ == "__main__":
    # mainSinTrajectory()
    # mainTrajectory()
    mainCircleTrajectory()