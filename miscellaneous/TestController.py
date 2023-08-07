import pinocchio as pin
from gepetto.corbaserver import Color
import numpy as np
from numpy.linalg import pinv, norm
import matplotlib.pyplot as plt
import time
import math
import sys
import os
from pinocchio.visualize import GepettoVisualizer
import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from trajectory import CircleTrajectory

"""
Pinocchio example to test Gepetto Viewer
"""

class TestController:

    def __init__(self):
        self.dt = 0.04
        # Load the URDF model. 
        urdf = os.path.abspath("urdf/sassa-robot/robot.urdf")
        model_path = os.path.abspath("urdf/sassa-robot/")
        self.model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf, model_path, pin.JointModelFreeFlyer())

        # for the new frame
        Z = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # np.eye(3)
        FIDX = self.model.getFrameId('OT')
        JIDX = self.model.frames[FIDX].parent
        eff = np.array([0.09, -0.008, 0.03])
        FIDX = self.model.addFrame(pin.Frame('framegripper', JIDX, FIDX, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

        self.data = self.model.createData()

        self.viz = GepettoVisualizer(model=self.model, collision_model=collision_model, visual_model=visual_model)
        
        # Initialize the viewer.
        try:
            self.viz.initViewer(loadModel=True)
        except ImportError as err:
            print("Error while initializing the viewer. It seems you should install gepetto-viewer")
            print(err)
            sys.exit(0)
        
        try:
            self.viz.loadViewerModel("pinocchio")
        except AttributeError as err:
            print("Error while loading the viewer model. It seems you should start gepetto-viewer")
            print(err)
            sys.exit(0)

        self.viz.viewer.gui.addSphere("world/pinocchio/goal", 0.01, Color.green)
        self.viz.viewer.gui.addSphere("world/pinocchio/gripper", 0.01, Color.red)
        
        # Display a robot configuration.
        self.q_current = pin.neutral(self.model)
        self.dq_current = np.zeros(self.model.nv)
        self.ddq_current = np.zeros(self.model.nv)

        # trajectory
        circle_trajectory = CircleTrajectory()
        circle_trajectory.circleTrajectoryXY(0.55, 0, 0.09, 0.02, 1)

        self.log_com = []
        self.log_goal = []
        self.log_end_effector = []

        for i in range(int(30 / self.dt)):
            t0 = time.time()

            goal = circle_trajectory.getPoint(i%360)
            self.q_current, self.dq_current, self.ddq_current = self.controller(goal)

            # log values
            self.log_com.append(pin.centerOfMass(self.model, self.data, self.q_current))
            self.log_goal.append(goal)
            IDX_Gripper = self.model.getFrameId('framegripper')
            frame_EF = [self.data.oMf[IDX_Gripper].homogeneous[:3, -1], \
                pin.getFrameVelocity(self.model, self.data, IDX_Gripper).vector[:3], np.array([0, 0, 0])]
            self.log_end_effector.append(frame_EF)

            if True:
                self.viz.display(self.q_current)
                a = np.hstack([goal[0], [0, 0, 0, 1]])
                self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal", list(a))
                self.viz.viewer.gui.refresh()
                tsleep = self.dt - (time.time() - t0)
                if tsleep > 0:
                    # wait to have a consitente frame rate
                    time.sleep(tsleep)


    def controller(self, goal):

        # Run the algorithms that outputs values in robot.data
        pin.forwardKinematics(self.model, self.data, self.q_current, self.dq_current, self.ddq_current)
        pin.framesForwardKinematics(self.model, self.data, self.q_current)
        pin.computeJointJacobians(self.model, self.data, self.q_current)

        # Gripper jacobian and error
        IDX_Gripper = self.model.getFrameId('framegripper')
        oMGripper = self.data.oMf[IDX_Gripper]
        
        a = np.hstack([oMGripper.translation, [0, 0, 0, 1]])
        self.viz.viewer.gui.applyConfiguration("world/pinocchio/gripper", list(a))

        o_JGripper = pin.computeFrameJacobian(self.model, self.data, self.q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)[:3,:]

        e_gripper = goal[0] - oMGripper.translation # target pos - current pos
        e_dot_gripper = goal[1] - (o_JGripper @ self.dq_current)
        
        a_gripper = pin.getFrameClassicalAcceleration(self.model, self.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
        # print(goal[2] + a_gripper[:3])

        # gains
        K1 = 1
        K2 = 2*np.sqrt(K1)

        ddq_next = pinv(o_JGripper) @ (goal[2] + a_gripper[:3] + K2 * e_dot_gripper + K1 * e_gripper)

        # compute the joints velocity
        dq_next = ddq_next * self.dt

        # compute the next configuration
        q_next = pin.integrate(self.model, self.q_current, dq_next * self.dt)

        # print(norm(e_gripper))

        return q_next, dq_next, ddq_next

    def plot(self, info):

        x_time_axis = np.arange(len(self.log_end_effector)) * 0.04

        if info == 1:
                # log test 1
                # Position error of the CoM
                fig = plt.figure()
                plt.subplot(3, 1, 1)
                e1 = [point[0] for point in self.log_com]
                plt.plot(x_time_axis, e1, label='X CoM position')
                plt.plot(x_time_axis, np.zeros(len(e1)), label='X CoM desired position', linestyle='dashed')
                plt.legend()
                # plt.ylim([-0.03, 0.042])
                plt.title("Sassa with long arm")
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.subplot(3, 1, 2)
                e2 = [point[1] for point in self.log_com]
                plt.plot(x_time_axis, e2, label='Y CoM position')
                plt.plot(x_time_axis, np.zeros(len(e2)), label='Y CoM desired position', linestyle='dashed')
                plt.legend()
                # plt.ylim([-0.01, 0.016])
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.subplot(3, 1, 3)
                e3 = [point[2] for point in self.log_com]
                plt.plot(x_time_axis, e3, label='Z CoM position')
                plt.legend()
                # plt.ylim([0.275, 0.4])
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.suptitle("title")
                fig.supxlabel("dt = 0.04 seconds")

                plt.subplots_adjust(left=0.125,
                    bottom=0.075,
                    right=0.9,
                    top=0.92,
                    wspace=0.45, # 0.2
                    hspace=0.37)

                plt.show()

        elif info == 2:
                # log test 1
                # Position error of the end effector
                fig = plt.figure()

                plt.subplot(3, 1, 1)
                e1 = [point[0][0]for point in self.log_end_effector]
                plt.plot(x_time_axis, e1, label='X end effector position')
                e1 = [point[0][0] for point in self.log_goal]
                plt.plot(x_time_axis, e1, label='X goal position', linestyle='dashed')
                plt.legend()
                # plt.ylim([0.3, 0.56])
                plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.subplot(3, 1, 2)
                e2 = [point[0][1] for point in self.log_end_effector]
                plt.plot(x_time_axis, e2, label='Y end effector position')
                e2 = [point[0][1] for point in self.log_goal]
                plt.plot(x_time_axis, e2, label='Y goal position', linestyle='dashed')
                plt.legend()
                plt.title("Position error on Y axis")
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.subplot(3, 1, 3)
                e3 = [point[0][2] for point in self.log_end_effector]
                plt.plot(x_time_axis, e3, label='Z end effector position')
                e3 = [point[0][2] for point in self.log_goal]
                plt.plot(x_time_axis, e3, label='Z goal position', linestyle='dashed')
                plt.legend()
                plt.title("Position error on Z axis")
                plt.xlabel("time (s)")
                plt.ylabel("meters")

                plt.suptitle("title")
                fig.supxlabel("dt = 0.04 seconds")
                # plt.subplot_tool()
                plt.subplots_adjust(left=0.125,
                        bottom=0.075,
                        right=0.9,
                        top=0.92,
                        wspace=0.45,
                        hspace=0.37)
                plt.show()

if __name__ == "__main__":
    my_controller = TestController()
    my_controller.plot(2)