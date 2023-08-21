import numpy as np
import pinocchio as pin
from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from gepetto.corbaserver import Color

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario1:

    def __init__(self, robot, viz, dt, q0_ref, control_point=None):
        """
        State machine to pick and place
        Also actuate the gripper 
        """
        self.robot = robot
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        self.current_state = 0
        if control_point is not None:
            self.control_point = control_point
        else:
            IDX_Gripper = self.robot.model.getFrameId('framegripper')
            frame_EF = self.robot.data.oMf[IDX_Gripper].homogeneous[:3, -1]
            self.control_point1 = [[0.35, 0.045, 0.4], [0.35, 0.05, 0.05]] #  frame_EF,
            self.control_point2 = [self.control_point1[-1], [0.35, 0.0, 0.11], [0.35, -0.05, 0.05]]
            self.control_point3 = [[0.35, -0.05, 0.05], [0.35, 0.0, 0.11], self.control_point1[-1]]
        self.end_time = 6
        self.end_time2 = 5
        init_vel = [0, 0, 0]
        end_vel = [0, 0, 0]
        init_acc = [0, 0, 0]
        end_acc = [0, 0, 0]
        self.trajectory1 = TrajectoryExactCubic(self.control_point1, 0, self.end_time, constraints=[init_acc, end_acc])
        self.trajectory2 = TrajectoryExactCubic(self.control_point2, 0, self.end_time2, constraints=[init_acc, end_acc])
        self.trajectory3 = TrajectoryExactCubic(self.control_point3, 0, self.end_time2, constraints=[init_acc, end_acc])
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
        return : new configuration and velocity vector to be displayed
        """


        if self.current_state == 0:
            # initial state
            # update q and dq here

            # go to the initial position
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory1.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            end_effector_rotation = 0 # ((np.pi/3) / ((self.end_time/3) / self.dt) ) * self.trajectory_i
            if self.trajectory_i > int((self.end_time/3) / self.dt):
                end_effector_rotation = ((np.pi/3) / (((self.end_time) - (self.end_time/3)) / self.dt) ) * self.trajectory_i

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', end_effector_rotation), eps=0.01)

            q, _ = actuate_gripper(self.robot, q, self.dt, action="open")

            self.init = False

            if self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 1
                self.init = True
                self.trajectory_i = 0


        elif self.current_state == 1:
            # when arrived, close the gripper
            q, dq, _ = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, action="close")

            self.init = False

            if task_finished:
                self.current_state = 2
                self.init = True


        elif self.current_state == 2:
            # go to end point while maintaining the gripper closed
            if self.trajectory_i > int(self.end_time2 / self.dt) - 1:
                self.trajectory_i = int(self.end_time2 / self.dt) - 1

            self.goal = self.trajectory2.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/2), eps=0.01) # np.pi/2

            self.init = False
            
            if self.trajectory_i >= int(self.end_time2 / self.dt) - 1:
                self.current_state = 3
                self.trajectory_i = 0
                self.init = True


        elif self.current_state == 3:
            # when at the place point, open the gripper
            q, dq, _ = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/3))
            q, task_finished = actuate_gripper(self.robot, q, self.dt, action="open")

            self.init = False

            if task_finished:
                self.current_state = 4
                self.trajectory_i = 0 # int(self.end_time2 / self.dt) - 1
                self.init = True

        
        elif self.current_state == 4:
            # go to first pick point
            if self.trajectory_i > int(self.end_time2 / self.dt) - 1:
                self.trajectory_i = int(self.end_time2 / self.dt) - 1

            self.goal = self.trajectory3.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', np.pi/3), eps=0.01)
            
            self.init = False
            
            if self.trajectory_i <= 0:
                self.current_state = 2
                self.trajectory_i = 0
                self.init = True

        if self.viz is not None:       
            a = self.goal[0]
            a = np.hstack([a, [0, 0, 0, 1]])
            self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal", list(a))
            self.viz.viewer.gui.refresh()

        return q, dq, self.goal