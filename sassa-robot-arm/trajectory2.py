import numpy as np
import time
import sys
import matplotlib.pyplot as plt
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
    def __init__(self, control_point=0, time_vector=0):
        """
        control_points : list of control points use for the trajectory
        """
        
        self.waypoints0 = np.array(control_point).transpose()
        self.waypoints1 = np.array(time_vector).transpose()
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
    def __init__(self, control_point, start_time, end_time, constraints=None):
        """
        control_points : list of control points use for the trajectory
        start_time : start time of the spline
        end_time : end time of the spline
        """
        self.waypoints = np.array(control_point).transpose() # control point of the curve
        self.start_time = start_time
        self.end_time = end_time
        self.time_waypoints = np.linspace(self.start_time, self.end_time, num=len(self.waypoints.T)).transpose() # uniforme time
        
        if constraints is None:
            self.ec = exact_cubic(self.waypoints, self.time_waypoints) # create the curve
        elif len(constraints) == 4:
            try:
                c = curve_constraints()
                c.init_vel = np.array(constraints[0]).transpose()
                c.end_vel = np.array(constraints[1]).transpose()
                c.init_acc = np.array(constraints[2]).transpose()
                c.end_acc = np.array(constraints[3]).transpose()

                self.ec = exact_cubic(self.waypoints, self.time_waypoints, c)
            except Exception as e:
                print("Constraint vector is not correct 1")
                print(e)
                sys.exit(0)
        elif len(constraints) == 2:
            try:
                c = curve_constraints()
                c.init_vel = np.array(constraints[0]).transpose()
                c.end_vel = np.array(constraints[1]).transpose()

                self.ec = exact_cubic(self.waypoints, self.time_waypoints, c)

            except Exception as e:
                print("Constraint vector is not correct 2")
                print(e)
                sys.exit(0)
        else:
            print("Constraint vector is not correct 3")
            sys.exit(0)


    def getPoint3d(self, i, dt): 
        # get the point position, velocity and acceleration
        return [self.ec(i*dt), self.ec.derivate(i*dt, 1), self.ec.derivate(i*dt, 2)]

    def getPoint6d(self, i, dt):
        return None

    def getAllPoint(self, step):
        liste = []
        for i in np.arange(self.start_time, self.end_time, step):
            liste.append(list(self.ec(i)))
        
        return list(liste)

    def printCurve(self, dt=0.04):
        # print the curve in 3D
        pos = []
        for i in range(int((self.time_waypoints[-1] - self.time_waypoints[0]) / dt)):
            pos.append(self.ec(i * dt))
            # pos.append(self.ec.derivate(i * dt, 2))
        
        x = [point[0] for point in pos]
        y = [point[1] for point in pos]
        z = [point[2] for point in pos]

        ax = plt.figure().add_subplot(projection='3d')
        ax.plot(x, y, z)
                
        ax.scatter([point[0] for point in self.waypoints.T], [point[1] for point in self.waypoints.T], [point[2] for point in self.waypoints.T])
        plt.show()

    def plotCurve(self, dt=0.04):
        # print the curve in 3D

        print(self.ec.getNumberSplines())

        pos = []
        vel = []
        acc = []
        axis = 0
        for i in range(int((self.time_waypoints[-1] - self.time_waypoints[0]) / dt)):
            pos.append(self.ec(i * dt)[axis])
            vel.append(self.ec.derivate(i * dt, 1)[axis])
            acc.append(self.ec.derivate(i * dt, 2)[axis])

        x_axis = np.arange(len(pos)) * dt
        
        plt.plot(x_axis, pos, label="position")
        plt.plot(x_axis, vel, label="velocity")
        plt.plot(x_axis, acc, label="acceleration")
        plt.xlabel("time in seconds")
        plt.ylabel("meters")
        plt.title("Piecewise polynomial curve with it's derivative")
        plt.legend()
        plt.show()


def mainTrajectory():
    control_points = [[0.35, -0.13, 0.25], [0.35, -0.13, 0.30], [0.4, 0.02, 0.45]]
    start_time = 0
    end_time = 10
    init_vel = [0, 0, 0]
    end_vel = [0, 0, 0]
    init_acc = [0, 0, 0]
    end_acc = [0, 0, 0]
    traj = TrajectoryExactCubic(control_points, start_time, end_time, constraints=[init_acc, end_acc])
    
    traj.printCurve(dt=0.04)
    traj.plotCurve()


if __name__ == "__main__":
    mainTrajectory()