import numpy as np
import pinocchio as pin
from gepetto.corbaserver import Color
import math

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory import SinTrajectory, CircleTrajectory
from trajectory2 import TrajectoryExactCubic
from controller import controllerCLIK2ndorder, controllerCLIK2ndorderBase
from gripper import actuate_gripper

class StateMahineScenario6:

    def __init__(self,robot, viz, dt, q0_ref, control_point=None):
        """
        State machine to pick and place
        Also actuate the gripper 
        """
        self.robot = robot
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        self.current_state = 0

        self.trajectory_x = SinTrajectory(2, 0.01)
        self.trajectory_y = SinTrajectory(4, 0.01)
        self.trajectory_z = SinTrajectory(8, 0.01)

        self.circle_trajectory = CircleTrajectory()
        self.circle_trajectory.circleTrajectoryYZ(0, 0, 0.3, 0.02, 8)

        z_offset = -0.1
        y_offset = 0.03
        self.control_point = [[0, 0, 0.4 + z_offset], [0, -y_offset, 0.38 + z_offset], [0, -y_offset, 0.42 + z_offset], \
                            [0, y_offset, 0.38 + z_offset], [0, y_offset, 0.42 + z_offset], [0, 0, 0.4 + z_offset]]
        self.end_time = 10
        self.trajectory = TrajectoryExactCubic(self.control_point, 0, self.end_time)

        self.init = True
        self.goal_base = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.goal_gripper = [[0.5, -0.015, 0.35], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.t0 = 0
        self.trajectory_i = 0

        if self.viz is not None:
            # to visualize the trajectory
            self.viz.viewer.gui.addSphere("world/pinocchio/goal_end_effector", 0.01, Color.yellow)
            self.viz.viewer.gui.addSphere("world/pinocchio/goal_base", 0.01, Color.green)
            


    def updateState(self, q, dq, i, add_goal_viz=True):
        """
        q : current robot contiguration vector
        dq : current robot joint velocity vector
        i : current state of the main loop
        add_goal_viz : add a sphere goal to represent the trajectory
        return : new configuration and velocity vector to be displayed
        """

        if self.current_state == 0:
            # initial state
            # go to lever
            # open gripper
            

            # self.goal_x = np.array(self.trajectory_x.getPoint3D(math.radians(i)))
            # self.goal_y = np.array(self.trajectory_y.getPoint3D(math.radians(i)))
            # self.goal_z = np.array(self.trajectory_z.getPoint3D(math.radians(i)))
            # self.goal_z[0] += 0.38
            # self.goal_base = np.concatenate([[self.goal_x.T], [self.goal_y.T], [self.goal_z.T]]).T
            # self.goal = self.circle_trajectory.getPoint(int(math.radians(i%360)))
            if self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.trajectory_i = 0

            self.goal_base = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorderBase(q, dq, self.dt, self.robot, self.init,\
                    self.viz, self.q0_ref, self.goal_gripper, self.goal_base, orientation=pin.utils.rotate('y', 0))

            self.init = False

            if 0:
                self.current_state = 1
                self.init = True


        elif self.current_state == 1:
            # close gripper
            
            q, task_finished = actuate_gripper(self.robot, q, self.dt, action="close")

            if task_finished:
                self.current_state = 2
                self.trajectory_i = 0

        if self.viz is not None:
            a = self.goal_base[0]
            a = np.hstack([a, [0, 0, 0, 1]])
            a = [float(i) for i in a]
            self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal_base", list(a))
            self.viz.viewer.gui.refresh()
            a = self.goal_gripper[0]
            a = np.hstack([a, [0, 0, 0, 1]])
            self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal_end_effector", list(a))
            self.viz.viewer.gui.refresh()

        return q, dq, self.goal_base