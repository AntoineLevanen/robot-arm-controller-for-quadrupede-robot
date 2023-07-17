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

    def __init__(self,robot, viz, dt, q0_ref, resolution=50):
        """
        State machine to pick and place
        Also actuate the gripper 
        """
        self.robot = robot
        self.resolution = resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        self.current_state = 0
        self.control_point = [[0.3, 0.1, 0.2], [0.3, 0.1, 0.25], [0.3, 0.0, 0.4], [0.3, -0.1, 0.25], [0.3, -0.1, 0.2]]
        self.trajectory = my_curve = Trajectory3D(self.control_point, generate_curve=True, resolution=self.resolution)
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

        print(self.current_state)

        if self.current_state == 0:
            # initial state
            # update q and dq here

            self.goal = self.trajectory.getPoint(24)

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.goal, self.q0_ref)

            self.init = False

            if task_finished:
                self.current_state = 1
                self.init = True
                self.trajectory_i = 24


        if self.current_state == 1:
            
            if  (i % 10) == 0 :
                # make sure to avoid Index out of range error (try, except...)
                self.goal = self.trajectory.getPoint(self.trajectory_i)
                self.trajectory_i = int(self.trajectory_i - 1)

            if self.trajectory_i < 0:
                self.goal = self.trajectory.getPoint(0)


            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.goal, self.q0_ref)
            q, _ = actuate_gripper(self.robot, q, self.dt, close=True)

            self.init = False
            
            if task_finished: ### on saute directement a la tache 3!!! car eps est < 0.012
                self.current_state = 2
                self.init = True


        elif self.current_state == 2:
            # close gripper
            q, task_finished = actuate_gripper(self.robot, q, self.dt, close=False)

            self.init = False

            if task_finished:
                self.current_state = 3
                self.init = True


        elif self.current_state == 3:
            pass


        return q, dq