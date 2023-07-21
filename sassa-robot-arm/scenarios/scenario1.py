import time
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from init import initRobot, initViz
from visualObject import CenterOfMass

from scenario1StateMachine import StateMahineScenario1


def scenario1(robot_urdf_path="urdf/sassa-robot/robot.urdf", robot_file_path="urdf/sassa-robot/", enable_viz=True):
    """
    Description:
    robot_urdf_path : path to urdf file
    robot_file_path : path to robot geometry file (STL, OBJ...)
    enable_viz : enable the visualizer

    """
    sassa = initRobot(robot_urdf_path, robot_file_path)
    # sassa = initRobot("urdf/sassa-robot-short-arm/robot.urdf", "urdf/sassa-robot-short-arm/")
    viz = None
    com_projection = None
    if enable_viz:
        viz = initViz(sassa, 2, add_ground=True, add_box=True, box_config=[0.4, 0.0, 0.04])
        # Object to show the projection on the ground of the center of masse 
        com_projection = CenterOfMass(viz, sassa, "com")
        viz.setCameraPose()

    duration = 60 # vizualization duration
    dt = 0.05 # delta time
    trajectory_step = int(duration / dt)

    # robot start configuration, velocity and acceleration
    q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                            np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])

    q_current = q0_ref.copy()

    dq_current = np.zeros((sassa.model.nv,))
    d2q_current = np.zeros((sassa.model.nv,))

    my_state_machine = StateMahineScenario1(sassa, viz, dt, q0_ref, curve_resolution=500)

    log_com = []
    log_goal = []
    log_end_effector = []

    # main loop, updating the configuration vector q
    for i in range(int(duration / dt)): # int(duration / dt)
        # start time for loop duration
        t0 = time.time()

        ### start controler

        # Implement the scenario here (State Machine, ...)
        q_current, dq_current, goal = my_state_machine.updateState(q_current, dq_current, i)

        ### end controler

        if enable_viz:
            # to display the movement in a 3D viewport
            viz.display(q_current)  
            viz.drawFrameVelocities(frame_id=sassa.model.getFrameId('framegripper'))
            rot_matrix = np.eye(4)
            rot_matrix[:3, :3] = pin.utils.rotate("z", np.pi)
            pos = sassa.data.oMf[sassa.model.getFrameId('framegripper')].homogeneous # @ rot_matrix
            viz.setCameraPose(pos)

        # log values
        if not enable_viz:
            log_com.append(pin.centerOfMass(sassa.model, sassa.data, q_current))
            log_goal.append(goal)
            IDX_Gripper = sassa.model.getFrameId('framegripper')
            frame_EF = [sassa.data.oMf[IDX_Gripper].homogeneous[:3, -1], \
                pin.getFrameVelocity(sassa.model, sassa.data, IDX_Gripper).vector[:3], np.array([0, 0, 0])]
            log_end_effector.append(frame_EF)
        
        
        # wait to have a real time sim
        if i % (1/dt) == 0 and enable_viz:
            # print the remaining time of the simulation in second
            print("time remaining :", duration-(i*dt))

        if enable_viz:
            tsleep = dt - (time.time() - t0)
            if tsleep > 0:
                # wait to have a consitente frame rate
                time.sleep(tsleep)


    return log_com, log_goal, log_end_effector


if __name__ == "__main__":
    log_com, log_goal, log_end_effector = scenario1(enable_viz=False)

    # plt.subplot(3, 1, 1)
    # e1 = [point[0] for point in log_com]
    # plt.plot(e1, label='X CoM position')
    # plt.plot(np.zeros(len(e1)), label='X CoM desired position')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # e2 = [point[1] for point in log_com]
    # plt.plot(e2, label='Y CoM position')
    # plt.plot(np.zeros(len(e2)), label='Y CoM desired position')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # e3 = [point[2] for point in log_com]
    # plt.plot(e3, label='Z CoM position')
    # plt.legend()

    # plt.show()