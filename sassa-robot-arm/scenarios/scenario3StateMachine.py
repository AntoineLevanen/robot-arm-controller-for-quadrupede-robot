import numpy as np
import pinocchio as pin
import time
import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from init import Cagette

from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario3:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50, control_point=None):
        """
        State machine to look inside a box and then move it
        Also actuate the gripper 
        """
        self.current_state = 0
        # if viz is not None:
        #     self.ma_cagette = Cagette(viz, initial_position=[0.4, 0.1, 0.1])
        # else:
        #     self.ma_cagette = None

        self.robot = robot
        self.curve_resolution = curve_resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        if control_point is not None:
            self.control_point = control_point
        else:
            self.control_point1 = [[0.48, -0.015, 0.45], [0.4, 0.02, 0.45], [0.35, 0.1, 0.25]]
            self.control_point2 = [[0.35, 0.1, 0.25], [0.35, -0.13, 0.25]]
            self.control_point3 = [[0.35, -0.13, 0.25], [0.35, -0.13, 0.30], [0.4, 0.02, 0.45]]
        self.end_time = 10
        self.end_time2 = 10
        init_vel = [0, 0, 0]
        end_vel = [0, 0, 0]
        init_acc = [0, 0, 0]
        end_acc = [0, 0, 0]
        self.trajectory1 = TrajectoryExactCubic(self.control_point1, 0, self.end_time, constraints=[init_vel, end_vel, init_acc, end_acc])
        self.trajectory2 = TrajectoryExactCubic(self.control_point2, 0, self.end_time2, constraints=[init_vel, end_vel, init_acc, end_acc])
        self.trajectory3 = TrajectoryExactCubic(self.control_point3, 0, self.end_time, constraints=[init_vel, end_vel, init_acc, end_acc])
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


    def updateState(self, q, dq, i, add_goal_viz=True):
        """
        q : current robot configuration vector
        dq : current robot joint velocity vector
        i : current state of the main loop
        return : new configuration and velocity vector to be displayed
        """

        # print(self.current_state)

        if self.current_state == 0:
            # go to first view point
            # update q and dq here
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory1.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1
            
            end_effector_rotation = ((np.pi/2) / (self.end_time / self.dt) ) * self.trajectory_i

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', end_effector_rotation), eps=0.01)

            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 1
                self.trajectory_i = 0
                self.init = True
                self.t0 = i


        elif self.current_state == 1:
            # wait 2 sec to take a picture
            if i - self.t0 > (2 / self.dt):
                self.current_state = 2


        elif self.current_state == 2:

            if self.trajectory_i > int(self.end_time2 / self.dt) - 1:
                self.trajectory_i = int(self.end_time2 / self.dt) - 1

            self.goal = self.trajectory2.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time2 / self.dt) - 1:
                self.current_state = 3
                self.trajectory_i = 0
                self.init = True
                self.t0 = i


        elif self.current_state == 3:
            # wait 1 sec to take a picture
            if i - self.t0 > (1 / self.dt):
                self.current_state = 4


        elif self.current_state == 4:
            # move the box
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory3.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1      

            end_effector_rotation = -((np.pi/2) / (self.end_time / self.dt) ) * self.trajectory_i + (np.pi/2)

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', end_effector_rotation))
            
            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 5
                self.init = True
                self.t0 = time.time()

        return q, dq, self.goal