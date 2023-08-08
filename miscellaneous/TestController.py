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
        self.dt = 0.01
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

        # to visualize the trajectory
        self.viz.viewer.gui.addSphere("world/pinocchio/goal", 0.01, Color.green)
        
        # current robot configuration and velocity
        q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0,\
                                    -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])
        q_current = q0_ref.copy() # pin.neutral(self.model)
        q_dot_current = np.zeros(self.model.nv)

        # trajectory
        circle_trajectory = CircleTrajectory()
        circle_trajectory.circleTrajectoryXY(0.559, -0.035, 0.457, 0.02, 1)

        self.log_goal = []
        self.log_end_effector = []

        for i in range(int(10 / self.dt)):
            t0 = time.time()


            goal = circle_trajectory.getPoint(i%360)

            q_current, q_dot_current = self.controller(q_current, q_dot_current, goal)


            # log values
            self.log_goal.append(goal)
            IDX_Gripper = self.model.getFrameId('framegripper')
            self.data.oMf[IDX_Gripper].homogeneous[:3, -1]
            self.log_end_effector.append(self.data.oMf[IDX_Gripper].homogeneous[:3, -1])

            if False: # visualize in gepetto viewer
                self.viz.display(q_current)

                goal_gepetto_configuration = np.hstack([goal[0], [0, 0, 0, 1]])
                self.viz.viewer.gui.applyConfiguration("world/pinocchio/goal", list(goal_gepetto_configuration))
                self.viz.viewer.gui.refresh()

                tsleep = self.dt - (time.time() - t0)
                if tsleep > 0:
                    # wait to have a consitente frame rate
                    time.sleep(tsleep)


    def controller(self, q, q_dot, goal):
        
        x_star = goal[0]
        x_star_dot = goal[1]
        x_star_ddot = goal[2]

        # Run the algorithms that outputs values in data
        pin.forwardKinematics(self.model, self.data, q, q_dot, q_dot * 0)
        pin.updateFramePlacements(self.model, self.data)

        # Gripper jacobian and error
        index_gripper = self.model.getFrameId('framegripper')     

        J_gripper = pin.computeFrameJacobian(self.model, self.data, q, index_gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]

        x = self.data.oMf[index_gripper] # get the current position of the end effector
        x_dot = (J_gripper @ q_dot)
        e = x_star - x.translation # position error
        e_dot = x_star_dot - x_dot
        
        Jdot_qdot = pin.getFrameClassicalAcceleration(self.model, self.data, index_gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

        # gains
        K1 = 1
        K2 = 2*np.sqrt(K1)

        q_ddot = pinv(J_gripper) @ (x_star_ddot - Jdot_qdot + K2 * e_dot + K1 * e)

        # compute the joints velocity
        q_dot = q_ddot * self.dt + q_dot

        # compute the next configuration
        q = pin.integrate(self.model, q, q_dot * self.dt)

        return q, q_dot

    def plot(self, info):

        x_time_axis = np.arange(len(self.log_end_effector)) * self.dt

        if info == 1:
            # log test 1
            # Position error of the end effector
            fig = plt.figure()

            plt.subplot(3, 1, 1)
            e1 = [point[0] for point in self.log_end_effector]
            plt.plot(x_time_axis, e1, label='X end effector position')
            e1 = [point[0][0] for point in self.log_goal]
            plt.plot(x_time_axis, e1, label='X goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 1, 2)
            e2 = [point[1] for point in self.log_end_effector]
            plt.plot(x_time_axis, e2, label='Y end effector position')
            e2 = [point[0][1] for point in self.log_goal]
            plt.plot(x_time_axis, e2, label='Y goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 1, 3)
            e3 = [point[2] for point in self.log_end_effector]
            plt.plot(x_time_axis, e3, label='Z end effector position')
            e3 = [point[0][2] for point in self.log_goal]
            plt.plot(x_time_axis, e3, label='Z goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Z axis")
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
            plt.subplot(3, 1, 1)
            e1 = [point[0] for point in self.log_end_effector]
            e2 = [point[0][0] for point in self.log_goal]
            mse_x = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, mse_x, label='X MSE')
            plt.legend()
            plt.title("Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("Mean square error")

            plt.subplot(3, 1, 2)
            e1 = [point[1] for point in self.log_end_effector]
            e2 = [point[0][1] for point in self.log_goal]
            mse_y = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, mse_y, label='Y MSE')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("Mean square error")

            plt.subplot(3, 1, 3)
            e1 = [point[2] for point in self.log_end_effector]
            e2 = [point[0][2] for point in self.log_goal]
            mse_z = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, mse_z, label='Z MSE')
            plt.legend()
            plt.title("Position error on Z axis")
            plt.xlabel("time (s)")
            plt.ylabel("Mean square error")

            plt.suptitle("Mean Square Error")
            plt.subplots_adjust(left=0.125,
                    bottom=0.075,
                    right=0.9,
                    top=0.92,
                    wspace=0.45, # 0.2
                    hspace=0.37)
           
            plt.show()

if __name__ == "__main__":
    my_controller = TestController()
    my_controller.plot(1) # trajectory
    # my_controller.plot(2) # mean square error