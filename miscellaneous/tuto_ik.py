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

mesh_dir_path = os.path.abspath("urdf/sassa-robot/")
urdf_model_path = os.path.abspath("urdf/sassa-robot/robot.urdf")
robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir_path, pin.JointModelFreeFlyer())
robot.initViewer(loadModel=True)
viz = initViz(robot, 1)
 
# create valid configuration
q = np.array([2.15015217e-09, -1.60740562e-11,  1.46242733e-09, -1.02385203e-11,
 -3.31462583e-10,  1.14100906e-11,  1.00000000e+00 , 0.00000000e+00,
  0.00000000e+00, 0.00000000e+00  ,0.00000000e+00,  0.00000000e+00,
  0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00,
  0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 ,-3.45269038e-12,
  4.99802238e-10 , 3.24310957e-10 , 1.48819677e-10 , 0.00000000e+00])
robot.display(q)

# for the new frame
Z = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # np.eye(3)
frame_index = robot.model.getFrameId('OT')
parent_frame_index = robot.model.frames[frame_index].parent
eff = np.array([0.09, -0.008, 0.03])
new_frame_index = robot.model.addFrame(pin.Frame('framegripper', parent_frame_index, frame_index, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

robot.data = robot.model.createData()

def place(name, M):
    """move frame in gepetto viewer"""
    robot.viewer.gui.applyConfiguration(name, pin.SE3ToXYZQUAT(M).tolist())
    robot.viewer.gui.refresh()

x_star_end_effector = pin.SE3(np.eye(3), np.array([0.58679037, -0.035, 0.09079037]))
robot.viewer.gui.addXYZaxis('world/framegoal', [1., 0., 0., 1.], .015, 0.1)
place('world/framegoal', x_star_end_effector)

log_goal = []
log_end_effector = []

enable_viz = True

dt = 0.001
duration = 10

# trajectory
circle_trajectory = CircleTrajectory(duration=duration, dt=dt)
# circle_trajectory.circleTrajectoryXY(0.58679037, -0.035, 0.09079037, 0.02, 1)

control_point = [[0.58679037, -0.015, 0.09079037], [0.7, -0.015, 0.09079037], [0.58679037, -0.015, 0.09079037], [0.7, 0.1, 0.1], [-2, -0.5, 3]]
c_vel = [0, 0, 0]
my_trajectory = TrajectoryExactCubic(control_point, 0, duration, constraints=[c_vel, c_vel, c_vel, c_vel])

if enable_viz == 1:
        viz.viewer.gui.addCurve("world/pinocchio/curve_1", my_trajectory.getAllPoint(dt), Color.lightBlue)

index_end_effector = robot.model.getFrameId('framegripper')
q_dot = np.zeros(robot.model.nv)

for i in range(int(duration / dt)):
    t0 = time.time()

    # pin.framesForwardKinematics(robot.model, robot.data, q) # Compute joint placements
    pin.forwardKinematics(robot.model, robot.data, q, q_dot, q_dot * 0)
    pin.computeJointJacobians(robot.model, robot.data, q) # Also compute operational frame placements

    # trajectory_point = circle_trajectory.getPoint(i%circle_trajectory.loop_duration)
    trajectory_point = my_trajectory.getPoint3d(i, dt)
    # trajectory_point = circle_trajectory.circleTrajectoryXY2(0.58679037, -0.035, 0.09079037, 0.02, i)
    
    trajectory_point_position = trajectory_point[0]
    trajectory_point_velocity = trajectory_point[1]
    trajectory_point_acceleration = trajectory_point[2]

    J_end_effector = pin.computeFrameJacobian(robot.model, robot.data, q, index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    Jdot_qdot_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    
    x_end_effector = robot.data.oMf[index_end_effector]  # Get placement from world frame o to frame f oMf
    x_star_end_effector = pin.SE3(np.eye(3), np.array(trajectory_point_position))
    x_dot_end_effector = J_end_effector @ q_dot
    x_star_dot_end_effector = trajectory_point_velocity

    x_star_ddot_end_effector = trajectory_point_acceleration

    error = x_star_end_effector.translation - x_end_effector.translation
    error_dot = x_star_dot_end_effector - x_dot_end_effector
    # embed()
    # q_dot = 1*pinv(J_end_effector) @ trajectory_point_velocity
    kp = 1
    kd = 2*np.sqrt(kp)
    q_ddot = pinv(J_end_effector) @ (x_star_ddot_end_effector - Jdot_qdot_gripper + kd * error_dot + kp * error)
    q_dot += q_ddot * dt
    
    q = pin.integrate(robot.model, q, q_dot * dt)
    
    log_goal.append(trajectory_point)
    log_end_effector.append(robot.data.oMf[index_end_effector].homogeneous[:3, -1])

    if enable_viz:
        robot.display(q)
        place('world/framegoal', x_star_end_effector)
        tsleep = dt - (time.time() - t0)
        if tsleep > 0:
            # wait to have a consitente frame rate
            time.sleep(tsleep)


x_time_axis = np.arange(len(log_end_effector)) * dt
plt.subplot(3, 1, 1)
e1 = [point[0] for point in log_end_effector]
plt.plot(x_time_axis, e1, label='X end effector position')
e1 = [point[0][0] for point in log_goal]
plt.plot(x_time_axis, e1, label='X desired position', linestyle='dashed')
plt.legend()
plt.title("Position error on X axis")
plt.xlabel("time (s)")
plt.ylabel("meters")

plt.subplot(3, 1, 2)
e2 = [point[1] for point in log_end_effector]
plt.plot(x_time_axis, e2, label='Y end effector position')
e2 = [point[0][1] for point in log_goal]
plt.plot(x_time_axis, e2, label='Y desired position', linestyle='dashed')
plt.legend()
plt.title("Position error on Y axis")
plt.xlabel("time (s)")
plt.ylabel("meters")

plt.subplot(3, 1, 3)
e3 = [point[2] for point in log_end_effector]
plt.plot(x_time_axis, e3, label='Z end effector position')
e3 = [point[0][2] for point in log_goal]
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
e1 = [point[0] for point in log_end_effector]
e2 = [point[0][0] for point in log_goal]
e_x = np.subtract(e2, e1)
plt.plot(x_time_axis, e_x, label='X axis error')
plt.legend()
# plt.ylim([0.3, 0.56])
plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
plt.xlabel("time (s)")
plt.ylabel("meter")

plt.subplot(3, 1, 2)
e1 = [point[1] for point in log_end_effector]
e2 = [point[0][1] for point in log_goal]
e_y = np.subtract(e2, e1)
plt.plot(x_time_axis, e_y, label='Y axis error')
plt.legend()
plt.title("Position error on Y axis")
plt.xlabel("time (s)")
plt.ylabel("meter")

plt.subplot(3, 1, 3)
e1 = [point[2] for point in log_end_effector]
e2 = [point[0][2] for point in log_goal]
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