import pinocchio as pin
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import os
from numpy.linalg import pinv
import time
import matplotlib.pyplot as plt
import sys
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from init import initViz
from trajectory import CircleTrajectory
from trajectory2 import TrajectoryExactCubic
# Starting gepetto server and give a time
import gepetto.corbaserver
from gepetto.corbaserver import Color
from IPython import embed
# gepetto.corbaserver.start_server()
# time.sleep(5)

class Controller:

    def __init__(self):
        mesh_dir_path = os.path.abspath("urdf/sassa-robot/")
        urdf_model_path = os.path.abspath("urdf/sassa-robot/robot.urdf")
        self.robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir_path, pin.JointModelFreeFlyer())
        self.robot.initViewer(loadModel=True)
        self.viz = initViz(self.robot, 1)
 
        # create valid configuration
        self.q = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                                    np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])
        self.robot.display(self.q)

        # for the new frame
        Z = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # np.eye(3)
        frame_index = self.robot.model.getFrameId('OT')
        parent_frame_index = self.robot.model.frames[frame_index].parent
        eff = np.array([0.09, -0.008, 0.03])
        new_frame_index = self.robot.model.addFrame(pin.Frame('framegripper', parent_frame_index, frame_index, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

        self.robot.data = self.robot.model.createData()

        x_star_end_effector = pin.SE3(np.eye(3), np.array([0.58679037, -0.035, 0.09079037]))
        self.robot.viewer.gui.addXYZaxis('world/framegoal', [1., 0., 0., 1.], .015, 0.1)
        self.place('world/framegoal', x_star_end_effector)

        self.log_goal = []
        self.log_end_effector = []

        self.dt = 0.001
        self.duration = 20

        # trajectory
        control_point = [[0.5586, -0.015, 0.4569], [0.5, -0.015, 0.44], [0.4, -0.1, 0.3], [0.4, 0.0, 0.4], [0.4, 0.1, 0.3], [0.5, -0.2, 0.4], [0.4, 0.2, 0.4]]
        c_vel = [0, 0, 0]
        self.my_trajectory = TrajectoryExactCubic(control_point, 0, self.duration) #, constraints=[c_vel, c_vel, c_vel, c_vel])

        self.viz.viewer.gui.addCurve("world/pinocchio/curve_1", self.my_trajectory.getAllPoint(self.dt), Color.lightBlue)

        self.index_end_effector = self.robot.model.getFrameId('framegripper')
        self.q_dot = np.zeros(self.robot.model.nv)

    def place(self, name, M):
        """move frame in gepetto viewer"""
        self.robot.viewer.gui.applyConfiguration(name, pin.SE3ToXYZQUAT(M).tolist())
        self.robot.viewer.gui.refresh()

    def controllerCLIK(self, enable_viz=True):
        for i in range(int(self.duration / self.dt)):
            t0 = time.time()

            # pin.framesForwardKinematics(robot.model, robot.data, q) # Compute joint placements
            pin.forwardKinematics(self.robot.model, self.robot.data, self.q, self.q_dot, self.q_dot * 0)
            pin.computeJointJacobians(self.robot.model, self.robot.data, self.q) # Also compute operational frame placements

            ### feet part
            # Get the robot frames
            index_fl_foot = self.robot.model.getFrameId('FL_foot_frame')
            index_fr_foot = self.robot.model.getFrameId('FR_foot_frame')
            index_hl_foot = self.robot.model.getFrameId('HL_foot_frame')
            index_hr_foot = self.robot.model.getFrameId('HR_foot_frame')

            # define the target frame of each foot
            feet_height = 0.016
            x_star_fl_foot = pin.SE3(np.eye(3), np.array([0.221, 0.140, feet_height]))
            x_star_fr_foot = pin.SE3(np.eye(3), np.array([0.221, -0.140, feet_height]))
            x_star_hl_foot = pin.SE3(np.eye(3), np.array([-0.221, 0.140, feet_height]))
            x_star_hr_foot = pin.SE3(np.eye(3), np.array([-0.221, -0.140, feet_height]))

            # compute feet position error and Jacobian
            x_fl_foot = self.robot.data.oMf[index_fl_foot] # get placement from world frame o to frame f
            # take only linear velocity
            J_fl_foot3 = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, index_fl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
            error_fl_foot = x_star_fl_foot.translation - x_fl_foot.translation # position error
            error_dot_fl_foot = np.zeros(3) - (J_fl_foot3 @ self.q_dot) # velocity error

            x_fr_foot = self.robot.data.oMf[index_fr_foot]
            J_fr_foot3 = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, index_fr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
            error_fr_foot = x_star_fr_foot.translation - x_fr_foot.translation
            error_dot_fr_foot = np.zeros(3) - (J_fr_foot3 @ self.q_dot)

            x_hl_foot = self.robot.data.oMf[index_hl_foot]
            J_hl_foot3 = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, index_hl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
            error_hl_foot = x_star_hl_foot.translation - x_hl_foot.translation
            error_dot_hl_foot = np.zeros(3) - (J_hl_foot3 @ self.q_dot)

            x_hr_foot = self.robot.data.oMf[index_hr_foot]
            J_hr_foot3 = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, index_hr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
            error_hr_foot = x_star_hr_foot.translation - x_hr_foot.translation
            error_dot_hr_foot = np.zeros(3) - (J_hr_foot3 @ self.q_dot)

            # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) in the 2ndCLIK equation
            Jdot_qdot_fl_foot = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, index_fl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            Jdot_qdot_fr_foot = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, index_fr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            Jdot_qdot_hl_foot = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, index_hl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            Jdot_qdot_hr_foot = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, index_hr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

            # Stack the different terme in vectors to have one task for all four feet
            error_feet = np.hstack([error_fl_foot, error_fr_foot, error_hl_foot, error_hr_foot])
            error_dot_feet = np.hstack([error_dot_fl_foot, error_dot_fr_foot, error_dot_hl_foot, error_dot_hr_foot])
            J_feet = np.vstack([J_fl_foot3, J_fr_foot3, J_hl_foot3, J_hr_foot3])
            Jdot_qdot_feet = np.hstack([Jdot_qdot_fl_foot, Jdot_qdot_fr_foot, Jdot_qdot_hl_foot, Jdot_qdot_hr_foot])
            # x_ddot_feet is indeed the term Jdot_qdot computed by Pinocchio
            x_star_ddot_feet = np.zeros(12)

            ### base part
            index_base = self.robot.model.getFrameId('body_sasm')
            J_base = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, index_base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
            Jdot_qdot_base = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, index_base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            x_base = self.robot.data.oMf[index_base]
            x_star_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
            x_dot_base = J_base @ self.q_dot
            x_star_dot_base = np.zeros(3)
            # x_ddot_base is indeed the term Jdot_qdot computed by Pinocchio
            x_star_ddot_base = np.zeros(3)

            error_base = x_star_base.translation - x_base.translation
            error_dot_base = x_star_dot_base - x_dot_base

            ### end effector part
            trajectory_point = self.my_trajectory.getPoint3d(i, self.dt)
            trajectory_point_position = trajectory_point[0]
            trajectory_point_velocity = trajectory_point[1]
            trajectory_point_acceleration = trajectory_point[2]

            orientation = pin.utils.rotate('y', np.pi/3)

            J_end_effector = pin.computeFrameJacobian(self.robot.model, self.robot.data, self.q, self.index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_end_effector = np.vstack([J_end_effector[:3,:], J_end_effector[4,:]])
            Jdot_qdot_end_effector = pin.getFrameClassicalAcceleration(self.robot.model, self.robot.data, self.index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
            Jdot_qdot_end_effector = np.hstack([Jdot_qdot_end_effector[:3], Jdot_qdot_end_effector[4]])
            
            x_end_effector = self.robot.data.oMf[self.index_end_effector] # Get placement from world frame o to frame f oMf
            x_star_end_effector = pin.SE3(orientation, np.array(trajectory_point_position))
            x_dot_end_effector = J_end_effector @ self.q_dot
            x_star_dot_end_effector = np.hstack([trajectory_point_velocity, 0])
            # x_ddot_end_effector is indeed the term Jdot_qdot computed by Pinocchio
            x_star_ddot_end_effector = np.hstack([trajectory_point_acceleration, 0])

            # error_end_effector = x_star_end_effector.translation - x_end_effector.translation
            error_end_effector = pin.log(x_end_effector.inverse() * x_star_end_effector).vector
            error_end_effector = np.hstack([error_end_effector[:3], error_end_effector[4]])
            error_dot_end_effector = x_star_dot_end_effector - x_dot_end_effector
            
            ### computing the acceleration vector
            kp = 10
            kd = 2*np.sqrt(kp)

            # combining end effector task with the feet one
            J = np.vstack([J_end_effector, J_feet])
            x_star_ddot = np.hstack([x_star_ddot_end_effector, x_star_ddot_feet])
            Jdot_qdot = np.hstack([Jdot_qdot_end_effector, Jdot_qdot_feet])
            error_dot = np.hstack([error_dot_end_effector, error_dot_feet])
            error = np.hstack([error_end_effector, error_feet])

            # end effector and feet task separated
            # q_ddot = pinv(J_end_effector) @ (x_star_ddot_end_effector - Jdot_qdot_end_effector + kd * error_dot_end_effector + kp * error_end_effector)
            # P0 = np.eye(robot.model.nv) - pinv(J_end_effector) @ J_end_effector
            # q_ddot += pinv(J_feet @ P0) @ (x_star_ddot_feet - Jdot_qdot_feet + kd * error_dot_feet + kp * error_feet)
            # P1 = P0 - pinv(J_feet @ P0) @ J_feet @ P0
            q_ddot = pinv(J) @ (x_star_ddot - Jdot_qdot + kd * error_dot + kp * error)
            P1 = np.eye(self.robot.model.nv) - pinv(J) @ J
            kp = 1
            kd = 2*np.sqrt(kp)
            q_ddot += pinv(J_base[:3] @ P1) @ (x_star_ddot_base[:3] - Jdot_qdot_base[:3] + kd * error_dot_base[:3] + kp * error_base[:3])

            # integrate the acceleration to get the velocity
            self.q_dot += q_ddot * self.dt
            
            # integrate the velocity to get the angular position of each joint
            self.q = pin.integrate(self.robot.model, self.q, self.q_dot * self.dt)
            
            # log some data to plot them
            self.log_goal.append(trajectory_point_position)
            self.log_end_effector.append(self.robot.data.oMf[self.index_end_effector].homogeneous[:3,-1])

            # to see the result in "real time" in gepetto viewer or just get the plot
            if enable_viz:
                self.robot.display(self.q)
                self.place('world/framegoal', x_star_end_effector)
                tsleep = self.dt - (time.time() - t0)
                if tsleep > 0:
                    # wait to have a consitente frame rate
                    time.sleep(tsleep)
        
        if enable_viz == 1:
            self.viz.viewer.gui.deleteNode("world/pinocchio/curve_1", True)


    def plotValues(self):
        x_time_axis = np.arange(len(self.log_end_effector)) * self.dt
        plt.subplot(3, 1, 1)
        e1 = [point[0] for point in self.log_end_effector]
        plt.plot(x_time_axis, e1, label='X end effector position')
        e1 = [point[0] for point in self.log_goal]
        plt.plot(x_time_axis, e1, label='X desired position', linestyle='dashed')
        plt.legend()
        plt.title("Position error on X axis")
        plt.xlabel("time (s)")
        plt.ylabel("meters")

        plt.subplot(3, 1, 2)
        e2 = [point[1] for point in self.log_end_effector]
        plt.plot(x_time_axis, e2, label='Y end effector position')
        e2 = [point[1] for point in self.log_goal]
        plt.plot(x_time_axis, e2, label='Y desired position', linestyle='dashed')
        plt.legend()
        plt.title("Position error on Y axis")
        plt.xlabel("time (s)")
        plt.ylabel("meters")

        plt.subplot(3, 1, 3)
        e3 = [point[2] for point in self.log_end_effector]
        plt.plot(x_time_axis, e3, label='Z end effector position')
        e3 = [point[2] for point in self.log_goal]
        plt.plot(x_time_axis, e3, label='Z desired position', linestyle='dashed')
        plt.legend()
        plt.title("Position error on Z axis")
        plt.xlabel("time (s)")
        plt.ylabel("meters")

        plt.suptitle(" ")
        plt.subplots_adjust(left=0.125,
                bottom=0.075,
                right=0.9,
                top=0.92,
                wspace=0.45, # 0.2
                hspace=0.37)

        plt.show()


        plt.subplot(3, 1, 1)
        e1 = [point[0] for point in self.log_end_effector]
        e2 = [point[0] for point in self.log_goal]
        e_x = np.subtract(e2, e1)
        plt.plot(x_time_axis, e_x, label='X axis error')
        plt.legend()
        # plt.ylim([0.3, 0.56])
        plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
        plt.xlabel("time (s)")
        plt.ylabel("meter")

        plt.subplot(3, 1, 2)
        e1 = [point[1] for point in self.log_end_effector]
        e2 = [point[1] for point in self.log_goal]
        e_y = np.subtract(e2, e1)
        plt.plot(x_time_axis, e_y, label='Y axis error')
        plt.legend()
        plt.title("Position error on Y axis")
        plt.xlabel("time (s)")
        plt.ylabel("meter")

        plt.subplot(3, 1, 3)
        e1 = [point[2] for point in self.log_end_effector]
        e2 = [point[2] for point in self.log_goal]
        e_z = np.subtract(e2, e1)
        plt.plot(x_time_axis, e_z, label='Z axis error')
        plt.legend()
        plt.title("Position error on Z axis")
        plt.xlabel("time (s)")
        plt.ylabel("meter")

        plt.suptitle(" ")
        # plt.subplot_tool()
        plt.subplots_adjust(left=0.125,
                bottom=0.075,
                right=0.9,
                top=0.92,
                wspace=0.45, # 0.2
                hspace=0.37)

        plt.show()


if __name__ == "__main__":
    my_controller = Controller()
    my_controller.controllerCLIK(enable_viz=False)
    my_controller.plotValues()