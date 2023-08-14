import numpy as np
import pinocchio as pin
from gepetto.corbaserver import Color
import time

from gripper import actuate_gripper

from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from trajectory2 import TrajectoryExactCubic

class StateMahineScenario2:

    def __init__(self,robot, viz, dt, q0_ref, control_point=None):
        """
        State machine to look over a table with a 2 nd order CLIK controller
        Also actuate the gripper 

        """
        self.current_state = -1

        self.robot = robot
        self.viz = viz
        self.dt = dt
        self.q0_ref = q0_ref
        if control_point is not None:
            self.control_point = control_point
        else:
            self.control_point1 = [[0.48, -0.015, 0.45], [0.4, -0.015, 0.45], [0.42, -0.015, 0.58]]
            self.control_point2 = [[0.42, -0.015, 0.58], [0.35, -0.015, 0.45], [0.48, -0.015, 0.45]]
        
        self.end_time = 8
        init_vel = [0, 0, 0]
        end_vel = [0, 0, 0]
        init_acc = [0, 0, 0]
        end_acc = [0, 0, 0]
        self.trajectory1 = TrajectoryExactCubic(self.control_point1, 0, self.end_time, constraints=[init_acc, end_acc])
        self.trajectory2 = TrajectoryExactCubic(self.control_point2, 0, self.end_time, constraints=[init_acc, end_acc])
        self.trajectory_i = 0
        self.init = True
        self.goal = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.t0 = 0
        # self.trajectory.printCurve()
        # init trajectory
        IDX_Gripper = self.robot.model.getFrameId('framegripper')
        frame_EF = self.robot.data.oMf[IDX_Gripper].homogeneous[:3, -1]
        self.trajectory0 = TrajectoryExactCubic([frame_EF, self.control_point1[0]], 0, 2, constraints=[init_vel, end_vel, init_acc, end_acc])

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

        # print(self.current_state)
        if self.current_state == -1:
            # initial state
            # update q and dq here

            # go to the initial position
            if self.trajectory_i > int(2 / self.dt) - 1:
                self.trajectory_i = int(2 / self.dt) - 1

            self.goal = self.trajectory0.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, _, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal,\
                                                 add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.01)

            self.init = False

            if self.trajectory_i >= int(2 / self.dt) - 1:
                self.current_state = 0
                self.init = True
                self.trajectory_i = 0


        elif self.current_state == 0:
            # initial state, go to end position
            # update q and dq here
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory1.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                    add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.0005)

            self.init = False

            if self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 1
                self.init = True
                self.trajectory_i = 0
                self.t0 = i


        elif self.current_state == 1:
            # wait 2 sec to take a picture
            
            if i - self.t0 > (2 / self.dt):
                self.current_state = 2


        if self.current_state == 2:
            # return to start position
            # make sure to avoid Index out of range error (try, except...)
            if self.trajectory_i > int(self.end_time / self.dt) - 1:
                self.trajectory_i = int(self.end_time / self.dt) - 1

            self.goal = self.trajectory2.getPoint3d(self.trajectory_i, self.dt)
            self.trajectory_i = self.trajectory_i + 1

            q, dq, task_finished = controllerCLIK2ndorder(q, dq, self.dt, self.robot, self.init, self.viz, self.q0_ref, self.goal, \
                                                    add_goal_sphere=add_goal_viz, orientation=pin.utils.rotate('y', 0), eps=0.0005)
            
            self.init = False
            
            if self.trajectory_i >= int(self.end_time / self.dt) - 1:
                self.current_state = 3
                self.trajectory_i = 0
                self.init = True
                self.t0 = i


        elif self.current_state == 3:
            # wait 4 sec to take a picture
            
            if i - self.t0 > (2 / self.dt):
                # start again to loop
                self.current_state = 0


        if self.viz is not None:       
            a = self.goal[0]
            a = np.hstack([a, [0, 0, 0, 1]])
            self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal", list(a))
            self.viz.viewer.gui.refresh()


        return q, dq, self.goal