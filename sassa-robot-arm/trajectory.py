import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline as Spline
from numpy import sin, cos
from geomdl import BSpline
from ndcurves import bezier# , piecewise_bezier_curve

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
    """
    Using the ndCurves library from robotpkg
    """
    def __init__(self, control_point=0, start_orientation=0, end_orientation=0):
        """
        control_points : list of control points use for the trajectory
        start_orientation : orientation of the end effector at the begining of the trajectory (Quaternion)
        end_orientation : orientation of the end effector at the end of the trajectory (Quaternion)
        """
        P0 = [1., 1., 1.]
        P1 = [2., 2., 2.]
        P2 = [3., 3., 3.]
        P3 = [4., 4., 4.]
        waypoints0 = matrix([P0, P1]).transpose()
        waypoints1 = matrix([P2, P3]).transpose()
        bc0 = bezier(waypoints0, 0., 1.)
        bc1 = bezier(waypoints1, 1., 3.)
        print(bc1(3.))
        pc = piecewise_bezier_curve()

        pc.add_curve(bc0)
        pc.add_curve(bc1)

        res = pc(0.)
        print(res)
        res = pc(3.)
        print(res)

        isC0 = pc.is_continuous(0)
        print(isC0)
        isC1 = pc.is_continuous(1)
        print(isC1)


        self.control_point = control_point
        self.start_orientation = start_orientation
        self.end_orientation = end_orientation
        # push back control points
        self.spline = None

    def getPoint3d(self, i):        
        return None

    def getPoint6d(self, i):
        return None

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

class Trajectory3D:
    
    def __init__(self, control_points, generate_curve=False, resolution=50, degree=3):
        self.control_points = control_points
        self.curve = BSpline.Curve()
        self.curve.degree = degree
        self.curve.ctrlpts = self.control_points
        self.curve_points = []
        cpt = 0
        for i in range(len(self.control_points) + 1 + self.curve.degree):
            if i < self.curve.degree + 1:
                self.curve.knotvector.append(0)
            elif i > len(self.control_points):
                self.curve.knotvector.append(cpt)
            else:
                cpt += 1
                self.curve.knotvector.append(cpt)

        self.curve.knotvector = [x / cpt for x in self.curve.knotvector]

        self.curve.delta = 1/resolution

        if generate_curve:
            self.generateTrajectory()

    def generateTrajectory(self):
        self.curve_points = self.curve.evalpts

    def getPoint(self, i):
        pos = np.array(self.curve_points[i]) # position
        try:
            vel = np.array(self.curve_points[i+1]) - pos # velocity, difference of position
            acc = (np.array(self.curve_points[i+2]) - np.array(self.curve_points[i+1])) - vel # acceleration, difference of velocity
        except IndexError:
            vel = np.array([0, 0, 0]) - pos # velocity, difference of position
            acc = np.array([0, 0, 0]) - vel # acceleration, difference of velocity

        return [pos, vel, acc]


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
    traj = Trajectory()
    
    time.sleep(200)
    result_list = []

    for i in range(10):
        a = traj.get_point_3d(i)
        print(a)
        result_list.append(a)


    plt.plot(result_list[:][0], result_list[:][1])
    plt.show()

def mainCircleTrajectory():
    """
    Seulement en position, pas d(iformation sur la vitesse ou l'acceleration)
    """
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

def mainTrajectory3D():
    """
    Seulement en position, pas d'information sur la vitesse ou l'acceleration!
    """
    # control_point = [[0.3, 0.1, 0.2], [0.3, 0.1, 0.25], [0.3, 0.0, 0.4], [0.3, -0.1, 0.25], [0.3, -0.1, 0.2]]
    aa = 0.35
    control_point = [[aa, 0.0, 0.4], [aa, 0.1, 0.35], [aa, 0.13, 0.22], [aa, 0.13, 0.17], [aa, 0.05, 0.17], \
                                [aa, -0.15, 0.17], [aa, -0.2, 0.17], [aa, -0.2, 0.2], [aa, -0.2, 0.3], [aa, 0.0, 0.4]]

    my_curve = Trajectory3D(control_point, generate_curve=True)

    err = []
    for i in range(len(my_curve.curve_points)):
        err.append(my_curve.getPoint(i)[0])

    x = [point[0] for point in err]
    y = [point[1] for point in err]
    z = [point[2] for point in err]

    ax = plt.figure().add_subplot(projection='3d')

    ax.plot(x, y, z, label='my 3D curve')
    ax.legend()

    ax.scatter([point[0] for point in control_point], [point[1] for point in control_point], [point[2] for point in control_point])

    plt.show()

def mainTrajectory3D_2():
    """
    Seulement en position, pas d'information sur la vitesse ou l'acceleration!
    """
    control_point = [[0, 0, 0], [10, 0, 10], [10, 10, 20], [20, 10, 30], [20, 0, 30], [30, 0, 20], [30, 10, 20], [40, 10, 10], [40, 0, 0]]

    my_curve = Trajectory3D(control_point, generate_curve=True, resolution=100, degree=5)

    err = []
    for i in range(len(my_curve.curve_points) - 2):
        err.append(my_curve.getPoint(i))
    pos = 1

    plt.subplot(3, 1, 1)
    e1 = [point[0][pos] for point in err]
    plt.plot(e1, label='position')
    plt.legend()

    plt.subplot(3, 1, 2)
    e2 = [point[1][pos] for point in err]
    plt.plot(e2, label="velocity")
    plt.legend()

    plt.subplot(3, 1, 3)
    e3 = [point[2][pos] for point in err]
    plt.plot(e3, label="acceleration")
    plt.legend()

    plt.show()

def mainTrajectory():
    """
    main for the Curve library from robotpkg
    """
    pass

if __name__ == "__main__":
    # mainSinTrajectory()
    mainTrajectory()
    # mainCircleTrajectory()
    # mainTrajectory3D()
    # mainTrajectory3D_2()