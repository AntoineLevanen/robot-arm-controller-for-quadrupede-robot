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
from trajectory import Trajectory3D
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario3:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50, control_point=None):
        """
        State machine to look inside a box and then move it
        Also actuate the gripper 
        """
        self.current_state = 0
        if viz is not None:
            self.ma_cagette = Cagette(viz, initial_position=[0.4, 0.1, 0.1])
        else:
            self.ma_cagette = None

        self.robot = robot
        self.curve_resolution = curve_resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        if control_point is not None:
            self.control_point = control_point
        else:
            self.control_point = [[0.35, 0.0, 0.4], [0.35, 0.13, 0.22], [0.35, 0.05, 0.17], [0.35, -0.15, 0.17], [0.35, -0.15, 0.2], [0.35, -0.1, 0.3]]

        self.end_time = 30
        self.trajectory = TrajectoryExactCubic(self.control_point, 0, self.end_time)
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
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            # self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == int(self.end_time * 5): ### trouver le point correspondant
                self.current_state = 1
                self.init = True
                self.t0 = i


        elif self.current_state == 1:
            # wait 2 sec to take a picture
            if i - self.t0 > (2 / self.dt):
                self.current_state = 2


        elif self.current_state == 2:

            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == int(self.end_time * 9.15): # 182: ### trouver le point correspondant
                self.current_state = 4
                self.init = True
                self.t0 = i


        elif self.current_state == 4:
            # move the box
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1
            box_actuate_start_time = int(self.end_time * 9.1)
            box_actuate_end_time = int(self.end_time * 14)          
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2))
            
            a = -0.2/(box_actuate_end_time - box_actuate_start_time)
            b = -0.1 - (-0.2/(box_actuate_end_time - box_actuate_start_time) * box_actuate_end_time)
            y = a * self.trajectory_i + b
            if self.viz is not None:
                self.ma_cagette.actuate(y=y)

            self.init = False

            if self.trajectory_i == box_actuate_end_time + 0:
                self.current_state = 5
                self.init = True
                self.t0 = time.time()

        
        elif self.current_state == 5:
            # move the box
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2), eps=0.029)

            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 6
                self.init = True
                self.t0 = i

        return q, dq, self.goal