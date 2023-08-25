import numpy as np
import pinocchio as pin
from gepetto.corbaserver import Color
from controller import controllerCLIK2ndorder
from gripper import actuate_gripper

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario5:

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
        
        x_offset = -0.06
        IDX_Gripper = self.robot.model.getFrameId('framegripper')
        frame_EF = self.robot.data.oMf[IDX_Gripper].homogeneous[:3, -1] # [0.55+x_offset, -0.015, 0.40]
        self.control_point1 = [frame_EF, [0.52+x_offset, 0.05, 0.45], [0.52+x_offset, 0.1, 0.45]]
        self.control_point2 = [self.control_point1[-1], [0.50+x_offset, 0.1, 0.40], [0.52+x_offset, 0.1, 0.35]]
        self.control_point3 = [self.control_point2[-1], [0.47+x_offset, 0.1, 0.35], [0.47+x_offset, -0.05, 0.45], \
                                                        [0.50+x_offset, -0.05, 0.45], [0.55+x_offset, -0.05, 0.45]]

        self.control_point4 = [self.control_point3[-1], [0.50+x_offset, -0.05, 0.45], [0.55+x_offset, -0.05, 0.45]]

        self.control_point5 = [[0.55+x_offset, -0.05, 0.45], [0.50+x_offset, -0.05, 0.45], self.control_point3[-1]]

        self.end_time1 = 6
        self.end_time2 = 8
        self.end_time3 = 6
        self.end_time4 = 10

        init_vel = [0, 0, 0]
        end_vel = [0, 0, 0]
        init_acc = [0, 0, 0]
        end_acc = [0, 0, 0]

        self.trajectory1 = TrajectoryExactCubic(self.control_point1, 0, self.end_time1, constraints=[init_vel, end_vel, init_acc, end_acc])
        self.trajectory2 = TrajectoryExactCubic(self.control_point2, 0, self.end_time2)
        self.trajectory3 = TrajectoryExactCubic(self.control_point3, 0, self.end_time3)# , constraints=[init_acc, end_acc])
        self.trajectory4 = TrajectoryExactCubic(self.control_point4, 0, self.end_time4, constraints=[init_acc, end_acc])
        self.trajectory5 = TrajectoryExactCubic(self.control_point5, 0, self.end_time4, constraints=[init_vel, end_vel, init_acc, end_acc])
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0


        if self.viz is not None:
            # to visualize the trajectory
            self.viz.viewer.gui.addSphere("world/pinocchio/goal", 0.01, Color.green)

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
            if self.trajectory_i > ((self.end_time1 / self.dt) - 1):
                self.trajectory_i = ((self.end_time1 / self.dt) - 1)

            self.goal = self.trajectory1.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.05)

            q, _ = actuate_gripper(self.robot, q, self.dt, action="open")

            self.init = False

            if self.trajectory_i >= int((self.end_time1 / self.dt) - 1):
                self.current_state = 1
                self.init = True
                self.trajectory_i = 0


        elif self.current_state == 1:
            # close gripper
            
            q, task_finished = actuate_gripper(self.robot, q, self.dt, action="close")

            if task_finished:
                self.current_state = 2
                self.trajectory_i = 0

        
        elif self.current_state == 2:
            # actuate lever
            if self.trajectory_i > ((self.end_time2 / self.dt) - 1):
                self.trajectory_i = ((self.end_time2 / self.dt) - 1)

            self.goal = self.trajectory2.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.0035)

            self.init = False

            if self.trajectory_i >= int(self.end_time2 / self.dt) - 1:
                self.current_state = 3
                self.init = True
                self.trajectory_i = 0

        elif self.current_state == 3:
            # open gripper
            
            q, task_finished = actuate_gripper(self.robot, q, self.dt, action="open")

            if task_finished:
                self.current_state = 4
                self.trajectory_i = 0

        
        elif self.current_state == 4:
            # leave lever area
            if self.trajectory_i > ((self.end_time3 / self.dt) - 1):
                self.trajectory_i = ((self.end_time3 / self.dt) - 1)

            self.goal = self.trajectory3.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.05)

            self.init = False

            if self.trajectory_i >= int(self.end_time3 / self.dt) - 1:
                self.current_state = 6
                self.init = True
                self.trajectory_i = 0

        
        elif self.current_state == 5:
            # go push button
            # close gripper
            if self.trajectory_i > ((self.end_time4 / self.dt) - 1):
                self.trajectory_i = ((self.end_time4 / self.dt) - 1)

            self.goal = self.trajectory4.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.003)

            q, _ = actuate_gripper(self.robot, q, self.dt, action="close")

            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time4 / self.dt):
                self.current_state = 6
                self.init = True
                self.trajectory_i = 0

        
        elif self.current_state == 6:
            # go back
            if self.trajectory_i > ((self.end_time4 / self.dt) - 1):
                self.trajectory_i = ((self.end_time4 / self.dt) - 1)

            self.goal = self.trajectory5.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.003)

            q , _ = actuate_gripper(self.robot, q, self.dt, action="close")

            self.init = False

            if task_finished and self.trajectory_i >= int(self.end_time4 / self.dt):
                self.current_state = 7
                self.init = True
                self.trajectory_i = 0


        if self.viz is not None:
            a = self.goal[0]
            a = np.hstack([a, [0, 0, 0, 1]])
            self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal", list(a))
            self.viz.viewer.gui.refresh()

        return q, dq, self.goal