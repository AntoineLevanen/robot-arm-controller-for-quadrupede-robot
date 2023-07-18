import numpy as np
import pinocchio as pin
import time

from gripper import actuate_gripper

from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from trajectory import Trajectory3D

class StateMahineScenario2:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50):
        """
        State machine to open a door with a 2 nd order CLIK controller
        Also actuate the gripper 

        """
        self.current_state = 0

        self.robot = robot
        self.curve_resolution = curve_resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        self.control_point = [[0.5, 0.0, 0.4], [0.3, 0.0, 0.6], [0.3, 0.0, 0.7], [0.45, 0.0, 0.7]]
        self.trajectory = my_curve = Trajectory3D(self.control_point, generate_curve=True, resolution=self.curve_resolution)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


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
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=True, orientation=pin.utils.rotate('y', 0), eps=0.03)

            self.init = False

            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 1
                self.init = True
                self.trajectory_i = self.curve_resolution - 1
                self.t0 = time.time()


        elif self.current_state == 1:
            # wait 2 sec to take a picture
            
            if time.time() - self.t0 > 4:
                self.current_state = 2


        if self.current_state == 2:
            # return to start position
            # make sure to avoid Index out of range error (try, except...)
            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i - 1

            if self.trajectory_i < 0:
                self.goal = self.trajectory.getPoint(0)

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=True, orientation=pin.utils.rotate('y', 0))
            
            self.init = False
            
            if task_finished and self.trajectory_i <= 0: ### on saute directement a la tache 3!!! car eps est < 0.012
                self.current_state = 3
                self.trajectory_i = 0
                self.init = True
                self.t0 = time.time()


        elif self.current_state == 3:
            # open gripper
            # wait 2 sec to take a picture
            
            if time.time() - self.t0 > 4:
                self.current_state = 0


        return q, dq