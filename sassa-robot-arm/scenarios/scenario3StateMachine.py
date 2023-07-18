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

class StateMahineScenario3:

    def __init__(self,robot, viz, dt, q0_ref, curve_resolution=50):
        """
        State machine to look inside a box and then move it
        Also actuate the gripper 
        """
        self.current_state = 0
        self.ma_cagette = Cagette(viz, initial_position=[0.4, 0.1, 0.0])

        self.robot = robot
        self.curve_resolution = curve_resolution
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        aa = 0.35
        self.control_point = [[aa, 0.0, 0.4], [aa, 0.1, 0.25], [aa, 0.13, 0.15], [aa, 0.13, 0.07], [aa, 0.05, 0.07], [aa, -0.15, 0.07], [aa, -0.2, 0.07], [aa, -0.2, 0.1], [aa, -0.2, 0.2], [aa, 0.0, 0.3]]
        self.trajectory = my_curve = Trajectory3D(self.control_point, generate_curve=True, resolution=self.curve_resolution)
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


    def updateState(self, q, dq, i):
        """
        q : current robot configuration vector
        dq : current robot joint velocity vector
        dt : delta time
        robot : robot wrapper class instance
        i : current state of the main loop
        viz : vizualizater to display goal point in 3D space
        return : new configuration and velocity vector to be displayed
        """

        # print(self.current_state)

        if self.current_state == 0:
            # go to first view point
            # update q and dq here
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint(self.trajectory_i)
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

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if self.trajectory_i == 200: ### trouver le point correspondant
                self.current_state = 3
                self.init = True
                self.t0 = time.time()


        elif self.current_state == 3:
            # wait 2 sec to take a picture
            
            if time.time() - self.t0 > 2:
                self.current_state = 6


        elif self.current_state == 6:
            # first contact with the box
            if self.trajectory_i > self.curve_resolution - 1:
                self.trajectory_i = self.curve_resolution - 1

            self.goal = self.trajectory.getPoint(self.trajectory_i)
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

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i + 1
            box_actuate_start_time = 235
            box_actuate_end_time = 380          
            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))
            
            a = -0.2/(box_actuate_end_time - box_actuate_start_time)
            b = -0.1 - (-0.2/(box_actuate_end_time - box_actuate_start_time) * box_actuate_end_time)
            y = a * self.trajectory_i + b
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

            self.goal = self.trajectory.getPoint(self.trajectory_i)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, add_goal_sphere=False, orientation=pin.utils.rotate('y', np.pi/2))

            self.init = False

            if task_finished:
                self.current_state = 9
                self.init = True
                self.t0 = time.time()


        return q, dq