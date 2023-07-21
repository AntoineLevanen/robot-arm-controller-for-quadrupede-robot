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
            aa = 0.35
            self.control_point = [[aa, 0.0, 0.4], [aa, 0.1, 0.35], [aa, 0.13, 0.22], [aa, 0.13, 0.17], [aa, 0.05, 0.17], \
                                [aa, -0.15, 0.17], [aa, -0.15, 0.17], [aa, -0.15, 0.2], [aa, -0.15, 0.3], [aa, 0.0, 0.3]]

            self.control_point = [[0.35, 0.0, 0.4], [0.35, 0.13, 0.22], [0.35, 0.05, 0.17], [0.35, -0.15, 0.17], [0.35, -0.15, 0.2], [0.35, 0.0, 0.3]]

        self.trajectory = Trajectory3D(self.control_point, generate_curve=True, resolution=self.curve_resolution)
        self.trajectory = TrajectoryExactCubic(self.control_point, 0, 20)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


    def updateState(self, q, dq, i):
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

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == 170: ### trouver le point correspondant
                self.current_state = 1
                self.init = True
                self.t0 = time.time()


        elif self.current_state == 1:
            # wait 2 sec to take a picture
            
            if time.time() - self.t0 > 2:
                self.current_state = 2


        elif self.current_state == 2:
            
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            # self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == 200: ### trouver le point correspondant
                self.current_state = 6
                self.init = True
                self.t0 = time.time()


        elif self.current_state == 6:
            # first contact with the box
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            # self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == 235:
                self.current_state = 7
                self.init = True
                self.t0 = time.time()


        elif self.current_state == 7:
            # move the box
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            # self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1
            box_actuate_start_time = 235
            box_actuate_end_time = 380          
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))
            
            a = -0.2/(box_actuate_end_time - box_actuate_start_time)
            b = -0.1 - (-0.2/(box_actuate_end_time - box_actuate_start_time) * box_actuate_end_time)
            y = a * self.trajectory_i + b
            if self.viz is not None:
                self.ma_cagette.actuate(y=y)

            self.init = False

            if self.trajectory_i == box_actuate_end_time + 0:
                self.current_state = 8
                self.init = True
                self.t0 = time.time()

        
        elif self.current_state == 8:
            # move the box
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            # self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.goal = self.trajectory.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2), eps=0.029)

            self.init = False

            if task_finished and self.trajectory_i >= self.curve_resolution - 1:
                self.current_state = 9
                self.init = True
                self.t0 = time.time()

        return q, dq, self.goal