import numpy as np
import pinocchio as pin
import time

from gripper import actuate_gripper

from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from trajectory import Trajectory3D
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario2:

    def __init__(self,robot, viz, dt, q0_ref, control_point=None):
        """
        State machine to look over a table with a 2 nd order CLIK controller
        Also actuate the gripper 

        """
        self.current_state = 0

        self.robot = robot
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        if control_point is not None:
            self.control_point = control_point
        else:
            self.control_point = [[0.5, -0.015, 0.4], [0.3, -0.015, 0.5], [0.3, -0.015, 0.6], [0.45, -0.015, 0.6]]
        
        self.end_time = 10
        self.trajectory = TrajectoryExactCubic(self.control_point, 0, self.end_time)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


    def updateState(self, q, dq, i):
        """
        q : current robot contiguration vector
        dq : current robot joint velocity vector
        i : current state of the main loop
        return : new configuration and velocity vector to be displayed
        """

        # print(self.current_state)

        if self.current_state == 0:
            # initial state, go to end position
            # update q and dq here
            if self.trajectory_i > ((self.end_time / self.dt) - 1):
                self.trajectory_i = ((self.end_time / self.dt) - 1)

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', 0), eps=0.03)

            self.init = False

            if task_finished and self.trajectory_i >= self.end_time / self.dt:
                print("End state 0")
                self.current_state = 1
                self.init = True
                self.trajectory_i = self.end_time / self.dt - 1
                self.t0 = i


        elif self.current_state == 1:
            # wait 4 sec to take a picture
            
            if i - self.t0 > (4 / self.dt):
                print("End state 1")
                print("wait 4 sec first")
                self.current_state = 2


        if self.current_state == 2:
            # return to start position
            # make sure to avoid Index out of range error (try, except...)
            if self.trajectory_i < 0:
                self.trajectory_i = 0

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i - 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', 0))
            
            self.init = False
            
            if task_finished and self.trajectory_i <= 0:
                print("End state 2")
                self.current_state = 3
                self.trajectory_i = 0
                self.init = True
                self.t0 = i


        elif self.current_state == 3:
            # wait 4 sec to take a picture
            
            if i - self.t0 > (4 / self.dt):
                # start again to loop
                print("End state 3")
                print("wait 4 sec last")
                self.current_state = 0


        return q, dq, self.goal