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
from trajectory import CircleTrajectory
# Starting gepetto server and give a time
import gepetto.corbaserver

gepetto.corbaserver.start_server()
# time.sleep(5)

mesh_dir_path = os.path.abspath("urdf/sassa-robot/")
urdf_model_path = os.path.abspath("urdf/sassa-robot/robot.urdf")
robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir_path, pin.JointModelFreeFlyer())
robot.initViewer(loadModel=True)
 
# create valid configuration
q = pin.neutral(robot.model)
robot.display(q)

# for the new frame
Z = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # np.eye(3)
frame_index = robot.model.getFrameId('OT')
parent_frame_index = robot.model.frames[frame_index].parent
eff = np.array([0.09, -0.008, 0.03])
new_frame_index = robot.model.addFrame(pin.Frame('framegripper', parent_frame_index, frame_index, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

robot.data = robot.model.createData()

# trajectory
circle_trajectory = CircleTrajectory()
circle_trajectory.circleTrajectoryXY(0.58679037, -0.035, 0.09079037, 0.02, 1)
 
pin.framesForwardKinematics(robot.model, robot.data, q)
end_effector_index = robot.model.getFrameId('framegripper')
x_end_effector = robot.data.oMf[end_effector_index]

def place(name, M):
    """move frame in gepetto viewer"""
    robot.viewer.gui.applyConfiguration(name, pin.SE3ToXYZQUAT(M).tolist())
    robot.viewer.gui.refresh()
 
def Rquat(x, y, z, w):
    q = pin.Quaternion(x, y, z, w)
    q.normalize()
    return q.matrix()


x_star_end_effector = pin.SE3(np.eye(3), np.array([0.58679037, -0.035, 0.09079037]))
robot.viewer.gui.addXYZaxis('world/framegoal', [1., 0., 0., 1.], .015, 0.1)
place('world/framegoal', x_star_end_effector)

log_goal = []
log_end_effector = []


dt = 0.01
q_dot = np.zeros(robot.model.nv)
for i in range(int(20 / dt)):
    t0 = time.time()

    pin.forwardKinematics(robot.model, robot.data, q)  # Compute joint placements
    pin.updateFramePlacements(robot.model, robot.data)      # Also compute operational frame placements

    x_end_effector = robot.data.oMf[end_effector_index]  # Get placement from world frame o to frame f oMf

    trajectory_point = circle_trajectory.getPoint(i%360)
    trajectory_point_position = trajectory_point[0]
    trajectory_point_velocity = trajectory_point[1]
    x_star_end_effector = pin.SE3(np.eye(3), np.array(trajectory_point_position))
    
    position_error = pin.log(x_end_effector.inverse() * x_star_end_effector).vector
    # position_error = x_end_effector.translation - x_star_end_effector.translation
    # x_dot = pin.getFrameVelocity(robot.model, robot.data, end_effector_index, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

    J_end_effector = pin.computeFrameJacobian(robot.model, robot.data, q, end_effector_index, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    
    q_dot = 1*pinv(J_end_effector) @ (position_error)

    q = pin.integrate(robot.model, q, q_dot * dt)

    log_goal.append(trajectory_point)
    log_end_effector.append(robot.data.oMf[end_effector_index].homogeneous[:3, -1])

    if 0:
        robot.display(q)
        place('world/framegoal', x_star_end_effector)
        tsleep = dt - (time.time() - t0)
        if tsleep > 0:
            # wait to have a consitente frame rate
            time.sleep(tsleep)

fig = plt.figure()
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
mse_x = np.square(np.subtract(e2, e1))
plt.plot(x_time_axis, mse_x, label='X MSE')
plt.legend()
# plt.ylim([0.3, 0.56])
plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
plt.xlabel("time (s)")
plt.ylabel("Mean square error")

plt.subplot(3, 1, 2)
e1 = [point[1] for point in log_end_effector]
e2 = [point[0][1] for point in log_goal]
mse_y = np.square(np.subtract(e2, e1))
plt.plot(x_time_axis, mse_y, label='Y MSE')
plt.legend()
plt.title("Position error on Y axis")
plt.xlabel("time (s)")
plt.ylabel("Mean square error")

plt.subplot(3, 1, 3)
e1 = [point[2] for point in log_end_effector]
e2 = [point[0][2] for point in log_goal]
mse_z = np.square(np.subtract(e2, e1))
plt.plot(x_time_axis, mse_z, label='Z MSE')
plt.legend()
plt.title("Position error on Z axis")
plt.xlabel("time (s)")
plt.ylabel("Mean square error")

plt.suptitle(" ")
# plt.subplot_tool()
plt.subplots_adjust(left=0.125,
        bottom=0.075,
        right=0.9,
        top=0.92,
        wspace=0.45, # 0.2
        hspace=0.37)

plt.show()