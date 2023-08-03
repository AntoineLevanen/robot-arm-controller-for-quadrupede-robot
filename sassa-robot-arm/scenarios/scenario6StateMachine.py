import numpy as np
import pinocchio as pin
import math

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory import SinTrajectory
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
        
        x_offset = -0.04
        self.control_point1 = [[0.55+x_offset, -0.015, 0.40], [0.47+x_offset, 0.0, 0.40], [0.47+x_offset, 0.1, 0.40], [0.52+x_offset, 0.1, 0.40]]

        self.end_time1 = 20


        self.trajectory_x = SinTrajectory(2, 0.05)
        self.trajectory_y = SinTrajectory(4, 0.05)
        self.trajectory_z = SinTrajectory(8, 0.05)

        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


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
            

            self.goal_x = np.array(self.trajectory_x.getPoint3D(math.radians(i)))
            self.goal_y = np.array(self.trajectory_y.getPoint3D(math.radians(i)))
            self.goal_z = np.array(self.trajectory_z.getPoint3D(math.radians(i)))
            self.goal_z[0] += 0.38
            self.goal = np.concatenate([[self.goal_x.T], [self.goal_y.T], [self.goal_z.T]]).T

            goal_gripper = [[0.4, -0.015, 0.35], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]

            q, dq, task_finished = controllerCLIK2ndorderBase(q, dq, self.dt, self.robot, self.init,\
                                                             self.viz, self.q0_ref, goal_gripper, base_task=self.goal, add_goal_sphere=False)

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



        return q, dq, self.goal