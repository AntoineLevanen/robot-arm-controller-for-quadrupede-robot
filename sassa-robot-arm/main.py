import time
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from init import initRobot, initViz
from controller import controllerCLIK2ndorderPositionOnly, controllerCLIK2ndorder
from gripper import actuate_gripper
from visualObject import CenterOfMass
from trajectory import CircleTrajectory, Trajectory3D

urdf_path = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa/robot_obj.urdf"
file_path = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa/"
sassa = initRobot(urdf_path, file_path)
viz = initViz(sassa, 2, add_ground=False, add_box=False)

duration = 60 # vizualization duration
dt = 0.05 # delta time
trajectory_step = int(duration / dt)
realtime_viz = True # 

q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])
# q0_ref = np.zeros(sassa.model.nq)


# q_current = np.zeros((sassa.model.nq,))
q_current = q0_ref.copy()

dq_current = np.zeros((sassa.model.nv,))
d2q_current = np.zeros((sassa.model.nv,))

init = True
is_close = True

# Object to show the projection on the ground of the center of masse 
# com_projection = CenterOfMass(viz, sassa, "com")

# State machine to implement a scenario of action
my_state_machine = StateMahine()

# circular trajectory
my_trajectory = CircleTrajectory()
# origine x, y, z, raduis, omega
my_trajectory.circleTrajectoryXY(0.45, -0.01, 0.3, 0.02, 1)

# B-Spline trajectory
control_points = [[0.4, 0.1, 0.2], [0.5, 0.0, 0.3], [0.5, -0.05, 0.5], [0.5, -0.1, 0.3], [0.5, 0.0, 0.6], [0.4, 0.1, 0.1]]
my_3d_trajectory = Trajectory3D(control_points, generate_curve=True, resolution=trajectory_step, degree=5)

goal = [[0.55, 0, 0.45], [0, 0, 0], [0, 0, 0]]
err = [[0, 0, 0]]

# main loop, updating the configuration vector q
for i in range(int(duration / dt)): # int(duration / dt)
    # start time for loop duration
    t0 = time.time()

    ### start controler

    # WORKING controller
    goal = my_trajectory.getPoint(i%360) # circular trajectory
    # goal = my_3d_trajectory.getPoint(i % trajectory_step) # 3D B-spline
    q_current, dq_current, _ = controllerCLIK2ndorder(q_current, dq_current, dt, sassa, init, viz, q0_ref, goal)

    # TEST controller
    # q, dq, _ = lookAt(q_current, dq_current, dt, sassa, i, viz, 1)
    # q, dq, _ = useGripper(q_current, dq_current, dt, sassa, i, viz)

    # STATE MACHINE
    # update the current state of the robot
    # q, dq = my_state_machine.updateState(q_current, dq_current, dt, sassa, i, viz)

    # ACTUATE gripper
    # q_current, is_gripper_end_actuated = actuate_gripper(sassa, q_current, dt, close=is_close)
    # if is_gripper_end_actuated:
    #     is_close = not is_close
    #     is_gripper_end_actuated = False

    init = False
    viz.display(q_current)
    # viz.drawFrameVelocities(sassa.model.getFrameId('framegripper'), v_scale=4)
    o_Gripper_frame = sassa.data.oMf[sassa.model.getFrameId('framegripper')].homogeneous
    viz.viewer.get_image()
    # time.sleep(100)

    # Update Geometry models
    sassa.updateGeometryPlacements(q_current, visual=False)

    # e = com_projection.updatePlacement(q_current)
    # err = np.vstack([err, e])
    ### end controler

    # wait to have a real time sim

    # if i % (1/dt) == 0:
    #     print("time remaining :", duration-(i*dt))

    if realtime_viz:
        tsleep = dt - (time.time() - t0)
        # print(tsleep)
        if tsleep > 0:
            time.sleep(tsleep)
            

plt.subplot(3, 1, 1)
e1 = [point[0] for point in err]
plt.plot(e1)

plt.subplot(3, 1, 2)
e2 = [point[1] for point in err]
plt.plot(e2)

plt.subplot(3, 1, 3)
e3 = [point[2] for point in err]
plt.plot(e3)

# plt.show()