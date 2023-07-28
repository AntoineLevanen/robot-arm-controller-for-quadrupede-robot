import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline as Spline
from numpy import sin, cos
from ndcurves import bezier, piecewise_bezier, exact_cubic, curve_constraints, polynomial

"""
Generate trajectory using the ndcurves lib
Bezier curve
ExactCubic, multiple polynomial that pass through each control point
"""

class TrajectoryBezier:
    """
    Using the ndCurves library from robotpkg
    Not implemented
    """
    def __init__(self, control_point=0, start_orientation=0, end_orientation=0):
        """
        control_points : list of control points use for the trajectory
        start_orientation : orientation of the end effector at the begining of the trajectory (Quaternion)
        end_orientation : orientation of the end effector at the end of the trajectory (Quaternion)
        """
        P0 = [1., 1., 0.]
        P1 = [1., 2., 0.]
        P2 = [2., 2., 0.]
        P3 = [2., 1., 0.]
        P4 = [2., 2., 0.]
        P5 = [2., 3., 0.]
        P6 = [3., 3., 0.]
        P7 = [3., 2., 0.]
        self.waypoints0 = np.array([P0, P1, P2, P3, P4, P5]).transpose()
        self.waypoints1 = np.array([P3, P5, P6, P7]).transpose()
        bc0 = bezier(self.waypoints0, 0., 1.)
        bc1 = bezier(self.waypoints1, 1., 3.)
        print(bc1(3.))
        self.pc = piecewise_bezier()
        self.dt = 0.01
        self.pc.append(bc0)
        self.pc.append(bc1)

        """ res = self.pc(0.)
        print(res)
        res = self.pc(3.)
        print(res)

        isC0 = self.pc.is_continuous(0)
        print(isC0)
        isC1 = self.pc.is_continuous(1)
        print(isC1) """


        self.control_point = control_point
        self.start_orientation = start_orientation
        self.end_orientation = end_orientation
        # push back control points
        self.spline = None

    def getPoint3d(self, i):        
        return None

    def getPoint6d(self, i):
        return None

    def printCurve(self):
        pos = []
        for i in range(200):
            pos.append(self.pc(i * self.dt))
        
        x = [point[0] for point in pos]
        y = [point[1] for point in pos]
        z = [point[2] for point in pos]

        ax = plt.figure().add_subplot(projection='3d')

        ax.plot(x, y, z)

        print(self.waypoints0.T)
                
        ax.scatter([point[0] for point in self.waypoints0.T], [point[1] for point in self.waypoints0.T], [point[2] for point in self.waypoints0.T])
        ax.scatter([point[0] for point in self.waypoints1.T], [point[1] for point in self.waypoints1.T], [point[2] for point in self.waypoints1.T])

        plt.show()

class TrajectoryExactCubic:
    """
    Using the ndCurves library from robotpkg
    """
    def __init__(self, control_point, start_time, end_time):
        """
        control_points : list of control points use for the trajectory
        start_time : start time of the spline
        end_time : end time of the spline
        """
        self.waypoints = np.array(control_point).transpose() # control point of the curve
        self.time_waypoints = np.linspace(start_time, end_time, num=len(self.waypoints.T)).transpose() # uniforme time
        # print(self.time_waypoints)
        self.ec = exact_cubic(self.waypoints, self.time_waypoints) # create the curve
        number_of_spline = self.ec.getNumberSplines() # number of sline used for the curve
        first_spline = self.ec.getSplineAt(0)


    def getPoint3d(self, i, dt): 
        # get the point position, velocity and acceleration       
        return [self.ec(i*dt), self.ec.derivate(i*dt, 1), self.ec.derivate(i*dt, 2)]

    def getPoint6d(self, i, dt):
        return None

    def printCurve(self, dt=0.01):
        # print the curve in 3D
        pos = []
        for i in range(int((self.time_waypoints[-1] - self.time_waypoints[0]) / dt)):
            pos.append(self.ec(i * dt))
        
        x = [point[0] for point in pos]
        y = [point[1] for point in pos]
        z = [point[2] for point in pos]

        ax = plt.figure().add_subplot(projection='3d')
        ax.plot(x, y, z)
                
        ax.scatter([point[0] for point in self.waypoints.T], [point[1] for point in self.waypoints.T], [point[2] for point in self.waypoints.T])
        plt.show()

def mainTrajectory():
    control_points = [[0.35, 0.0, 0.4], [0.35, 0.13, 0.22], [0.35, 0.05, 0.17], [0.35, -0.15, 0.17], [0.35, -0.15, 0.2], [0.35, 0.0, 0.3]]
    control_points = [[0.4, 0.1, 0.2], [0.4, 0.1, 0.25], [0.4, 0.0, 0.4], [0.4, -0.1, 0.25], [0.4, -0.1, 0.2]]
    control_points = [[0.5, 0.0, 0.4], [0.3, 0.0, 0.5], [0.3, 0.0, 0.6], [0.45, 0.0, 0.6]]
    start_time = 0
    end_time = 10
    traj = TrajectoryExactCubic(control_points, start_time, end_time)
    
    traj.printCurve(dt=0.01)


if __name__ == "__main__":
    mainTrajectory()