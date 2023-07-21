import numpy as np
import pinocchio as pin
from controller import controllerCLIK2ndorder
from gripper import actuate_gripper

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory import Trajectory3D
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario1:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50, control_point=None):
        """
        State machine to pick and place
        Also actuate the gripper 
        """
        self.robot = robot
        self.curve_resolution = curve_resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        self.current_state = 0
        if control_point is not None:
            self.control_point = control_point
        else:
            self.control_point = [[0.4, 0.1, 0.2], [0.4, 0.1, 0.25], [0.4, 0.0, 0.4], [0.4, -0.1, 0.25], [0.4, -0.1, 0.2]]
        self.end_time = 10
        self.trajectory = TrajectoryExactCubic(self.control_point, 0, self.end_time)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]


    def updateState(self, q, dq, i):
        """
        q : current robot contiguration vector
        dq : current robot joint velocity vector
        i : current state of the main loop
        return : new configuration and velocity vector to be displayed
        """

        # print(self.current_state)

        if self.current_state == 0:
            # initial state
            # update q and dq here

            # go to the initial position
            self.goal = self.trajectory.getPoint3d(int(self.end_time / 2), self.dt)

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/6), eps=0.03)

            self.init = False

            if task_finished:
                self.current_state = 1
                self.init = True
                self.trajectory_i = int(self.end_time / 2)


        if self.current_state == 1:
            # go to first pick point and open the gripper
            # make sure to avoid Index out of range error (try, except...)
            if self.trajectory_i < 0:
                self.trajectory_i = 0

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i - 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/3))
            q, _ = actuate_gripper(self.robot, q, self.dt, close=True)

            self.init = False
            
            if task_finished and self.trajectory_i <= 0:
                self.current_state = 2
                self.trajectory_i = 0
                self.init = True


        elif self.current_state == 2:
            # when arrived, close the gripper
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=False)

            self.init = False

            if task_finished:
                self.current_state = 3
                self.init = True


        elif self.current_state == 3:
            # go to end point while maintaining the gripper closed
            if self.trajectory_i > self.end_time / self.dt - 1:
                self.trajectory_i = self.end_time / self.dt - 1

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, eps=0.035, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False
            
            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 4
                self.trajectory_i = self.end_time / self.dt - 1
                self.init = True


        elif self.current_state == 4:
            # when at the place point, open the gripper
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, eps=0.035, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/3))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=True)

            self.init = False

            if task_finished:
                self.current_state = 5
                self.init = True


        elif self.current_state == 5:
            # go to waiting point
            if self.trajectory_i < int(self.end_time / 2):
                self.trajectory_i = int(self.end_time / 2)

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            #self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i - 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, orientation=pin.utils.rotate('y', np.pi/3), add_goal_sphere=False)
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=True)
            
            self.init = False
            
            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 0
                self.trajectory_i = int(self.end_time / 2)
                self.init = True

        return q, dq, self.goal