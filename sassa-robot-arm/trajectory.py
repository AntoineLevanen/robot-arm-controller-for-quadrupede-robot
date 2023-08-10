import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline as Spline
from numpy import sin, cos

"""
Generate circle and sin trajectory
"""

class SinTrajectory:

    def __init__(self, omega, amplitude):
        self.amplitude = amplitude
        self.omega = omega # frequency

    def pos(self, t):
        return self.amplitude * np.sin(self.omega * t)

    def vel(self, t):
        return self.omega * self.amplitude * np.cos(self.omega * t)

    def acc(self, t):
        return -self.omega**2 * self.amplitude * np.sin(self.omega * t)

    def getPoint3D(self, i):
        return [self.pos(i), self.vel(i), self.acc(i)]

class CircleTrajectory:

    def __init__(self, duration=1, dt=0.01):
        self.x = []
        self.y = []
        self.z = []
        self.dx = []
        self.dy = []
        self.dz = []
        self.d2x = []
        self.d2y = []
        self.d2z = []
        self.loop_duration = int(duration / dt) 

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
        a = 360 / self.loop_duration
        for i in range(self.loop_duration):
            self.x.append(x_center + raduis * sin(omega * d2r(a * i)))
            self.y.append(y_center + raduis * cos(omega * d2r(a * i)))
            self.z.append(z_center)

            self.dx.append(omega * raduis * cos(omega * d2r(a * i)))
            self.dy.append(- omega * raduis * sin(omega * d2r(a * i)))
            self.dz.append(0)

            self.d2x.append(- omega**2 * raduis * sin(omega * d2r(a * i)))
            self.d2y.append(- omega**2 * raduis * cos(omega * d2r(a * i)))
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
    qdes = SinTrajectory(np.array([0, 0, 0]), omega=20, amplitude=0.3)
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

def mainCircleTrajectory():
    """
    Seulement en position, pas d(iformation sur la vitesse ou l'acceleration)
    """
    my_trajectory = CircleTrajectory(duration=4, dt=0.01)
    my_trajectory.circleTrajectoryXY(0.4, -0.05, 0.2, 0.14, 1)

    dt = 0.01
    idx = 0

    err1x = []
    err1y = []
    err2x = []
    err2y = []
    err3x = []
    err3y = []


    for i in range(my_trajectory.loop_duration):
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
    mainCircleTrajectory()