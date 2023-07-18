import numpy as np
import pinocchio as pin
from controller import controllerCLIK2ndorder
from gripper import actuate_gripper

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory import Trajectory3D

class StateMahineScenario1:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50):
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
        self.control_point = [[0.4, 0.1, 0.2], [0.4, 0.1, 0.25], [0.4, 0.0, 0.4], [0.4, -0.1, 0.25], [0.4, -0.1, 0.2]]
        self.trajectory = my_curve = Trajectory3D(self.control_point, generate_curve=True, resolution=self.curve_resolution)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]


    def updateState(self, q, dq, i):
        """
        q : current robot contiguration vector
        dq : current robot joint velocity vector
        dt : delta time
        robot : robot wrapper class instance
        i : current state of the main loop
        viz : vizualizater to display goal point in 3D space
        return : new configuration and velocity vector to be displayed
        """

        # print(self.current_state)

        if self.current_state == 0:
            # initial state
            # update q and dq here

            self.goal = self.trajectory.getPoint(int(self.curve_resolution / 2)) # point de parking, milieu de la trajectoire

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/6))

            self.init = False

            if task_finished:
                self.current_state = 1
                self.init = True
                self.trajectory_i = int(self.curve_resolution / 2)


        if self.current_state == 1:
            # go to first point
            # make sure to avoid Index out of range error (try, except...)
            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i - 1

            if self.trajectory_i < 0:
                self.goal = self.trajectory.getPoint(0)


            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/3))
            q, _ = actuate_gripper(self.robot, q, self.dt, close=True)

            self.init = False
            
            if task_finished and self.trajectory_i <= 0: ### on saute directement a la tache 3!!! car eps est < 0.012
                self.current_state = 2
                self.trajectory_i = 0
                self.init = True


        elif self.current_state == 2:
            # open gripper
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=False)

            self.init = False

            if task_finished:
                self.current_state = 3
                self.init = True


        elif self.current_state == 3:
            # go to end point
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, eps=0.035, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False
            
            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 4
                self.trajectory_i = self.curve_resolution - 1
                self.init = True


        elif self.current_state == 4:
            # close gripper
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, eps=0.035, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/3))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=True)

            self.init = False

            if task_finished:
                self.current_state = 5
                self.init = True


        elif self.current_state == 5:
            # go to waiting point
            if self.trajectory_i < int(self.curve_resolution / 2):
                self.trajectory_i = int(self.curve_resolution / 2)

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i - 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, orientation=pin.utils.rotate('y', np.pi/3), add_goal_sphere=False)
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=True)
            
            self.init = False
            
            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 6
                self.trajectory_i = int(self.curve_resolution / 2)
                self.init = True


        return q, dq