import time
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
from gepetto.corbaserver import Color
import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from init import initRobot, initViz
from visualObject import CenterOfMass

from scenario1StateMachine import StateMahineScenario1


def scenario1(robot_urdf_path="urdf/sassa-robot/robot.urdf", robot_file_path="urdf/sassa-robot/", enable_viz=True, export_to_blender=False):
    """
    Description:
    robot_urdf_path : path to urdf file
    robot_file_path : path to robot geometry file (STL, OBJ...)
    enable_viz : enable the visualizer, 1 Gepetto Viewer, 2 Meshcat, False

    """
    sassa = initRobot(robot_urdf_path, robot_file_path)
    # sassa = initRobot("urdf/sassa-robot-short-arm/robot.urdf", "urdf/sassa-robot-short-arm/")
    viz = None
    com_projection = None
    visual_object = False
    if enable_viz == 1:
        viz = initViz(sassa, 1, add_ground=visual_object, add_box=visual_object)
        viz.viewer.gui.addSphere("world/pinocchio/CoM", 0.01, Color.green)
        
    elif enable_viz == 2:
        visual_object = True
        viz = initViz(sassa, 2, add_ground=visual_object, add_box=visual_object, box_config=[0.4, 0.0, 0.04])
        # Object to show the projection on the ground of the center of masse 
        com_projection = CenterOfMass(viz, sassa, "com")
    else:
        enable_viz = False
        
    duration = 30 # vizualization duration
    dt = 0.001 # delta time

    # robot start configuration, velocity
    q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                            np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])

    q_current = q0_ref.copy() # pin.neutral(sassa.model)

    dq_current = np.zeros((sassa.model.nv,))

    sassa.forwardKinematics(q_current)
    pin.updateFramePlacements(sassa.model, sassa.data)
    if enable_viz:
        viz.display(q_current)

    my_state_machine = StateMahineScenario1(sassa, viz, dt, q0_ref)

    log_com = []
    log_goal = []
    log_end_effector = []

    # capture to export to blender
    node_name = [
        "world/pinocchio/collisions/body_sasm_0", 
        "world/pinocchio/collisions/body_sasm_1",
        "world/pinocchio/collisions/body_sasm_2",
        "world/pinocchio/collisions/body_sasm_3",
        "world/pinocchio/collisions/body_sasm_4",
        "world/pinocchio/collisions/body_sasm_5",
        "world/pinocchio/collisions/body_sasm_6",
        "world/pinocchio/collisions/body_sasm_7",
        "world/pinocchio/collisions/helbow_sasm_0",
        "world/pinocchio/collisions/helbow_sasm_1",
        "world/pinocchio/collisions/upperleg_sasm_0",
        "world/pinocchio/collisions/upperleg_sasm_1",
        "world/pinocchio/collisions/upperleg_sasm_2",
        "world/pinocchio/collisions/upperleg_sasm_3",
        "world/pinocchio/collisions/upperleg_sasm_4",
        "world/pinocchio/collisions/upperleg_sasm_5",
        "world/pinocchio/collisions/lowerleg_sasm_0",
        "world/pinocchio/collisions/lowerleg_sasm_1",
        "world/pinocchio/collisions/lowerleg_sasm_2",
        "world/pinocchio/collisions/lowerleg_sasm_3",
        "world/pinocchio/collisions/helbow_sasm_2_0",
        "world/pinocchio/collisions/helbow_sasm_2_1",
        "world/pinocchio/collisions/upperleg_sasm_2_0",
        "world/pinocchio/collisions/upperleg_sasm_2_1",
        "world/pinocchio/collisions/upperleg_sasm_2_2",
        "world/pinocchio/collisions/upperleg_sasm_2_3",
        "world/pinocchio/collisions/upperleg_sasm_2_4",
        "world/pinocchio/collisions/upperleg_sasm_2_5",
        "world/pinocchio/collisions/lowerleg_sasm_2_0",
        "world/pinocchio/collisions/lowerleg_sasm_2_1",
        "world/pinocchio/collisions/lowerleg_sasm_2_2",
        "world/pinocchio/collisions/lowerleg_sasm_2_3",
        "world/pinocchio/collisions/helbow_sasm_3_0", 
        "world/pinocchio/collisions/helbow_sasm_3_1", 
        "world/pinocchio/collisions/upperleg_sasm_3_0", 
        "world/pinocchio/collisions/upperleg_sasm_3_1", 
        "world/pinocchio/collisions/upperleg_sasm_3_2", 
        "world/pinocchio/collisions/upperleg_sasm_3_3", 
        "world/pinocchio/collisions/upperleg_sasm_3_4", 
        "world/pinocchio/collisions/upperleg_sasm_3_5", 
        "world/pinocchio/collisions/lowerleg_sasm_3_0", 
        "world/pinocchio/collisions/lowerleg_sasm_3_1", 
        "world/pinocchio/collisions/lowerleg_sasm_3_2", 
        "world/pinocchio/collisions/lowerleg_sasm_3_3", 
        "world/pinocchio/collisions/helbow_sasm_4_0", 
        "world/pinocchio/collisions/helbow_sasm_4_1", 
        "world/pinocchio/collisions/upperleg_sasm_4_0", 
        "world/pinocchio/collisions/upperleg_sasm_4_1", 
        "world/pinocchio/collisions/upperleg_sasm_4_2", 
        "world/pinocchio/collisions/upperleg_sasm_4_3", 
        "world/pinocchio/collisions/upperleg_sasm_4_4", 
        "world/pinocchio/collisions/upperleg_sasm_4_5", 
        "world/pinocchio/collisions/lowerleg_sasm_4_0", 
        "world/pinocchio/collisions/lowerleg_sasm_4_1", 
        "world/pinocchio/collisions/lowerleg_sasm_4_2", 
        "world/pinocchio/collisions/lowerleg_sasm_4_3", 
        "world/pinocchio/collisions/arm1_sasm_0", 
        "world/pinocchio/collisions/arm2_sasm_0", 
        "world/pinocchio/collisions/arm2_sasm_1", 
        "world/pinocchio/collisions/arm2_sasm_2", 
        "world/pinocchio/collisions/arm3_sasm_0", 
        "world/pinocchio/collisions/end_effector_sasm_0", 
        "world/pinocchio/collisions/end_effector_sasm_1", 
        "world/pinocchio/collisions/end_effector_sasm_2", 
        "world/pinocchio/collisions/gripper_0", 
        "world/pinocchio/visuals/body_sasm_0", 
        "world/pinocchio/visuals/body_sasm_1", 
        "world/pinocchio/visuals/body_sasm_2", 
        "world/pinocchio/visuals/body_sasm_3", 
        "world/pinocchio/visuals/body_sasm_4", 
        "world/pinocchio/visuals/body_sasm_5", 
        "world/pinocchio/visuals/body_sasm_6", 
        "world/pinocchio/visuals/body_sasm_7", 
        "world/pinocchio/visuals/helbow_sasm_0", 
        "world/pinocchio/visuals/helbow_sasm_1", 
        "world/pinocchio/visuals/upperleg_sasm_0", 
        "world/pinocchio/visuals/upperleg_sasm_1", 
        "world/pinocchio/visuals/upperleg_sasm_2", 
        "world/pinocchio/visuals/upperleg_sasm_3", 
        "world/pinocchio/visuals/upperleg_sasm_4", 
        "world/pinocchio/visuals/upperleg_sasm_5", 
        "world/pinocchio/visuals/lowerleg_sasm_0", 
        "world/pinocchio/visuals/lowerleg_sasm_1", 
        "world/pinocchio/visuals/lowerleg_sasm_2", 
        "world/pinocchio/visuals/lowerleg_sasm_3", 
        "world/pinocchio/visuals/helbow_sasm_2_0", 
        "world/pinocchio/visuals/helbow_sasm_2_1", 
        "world/pinocchio/visuals/upperleg_sasm_2_0", 
        "world/pinocchio/visuals/upperleg_sasm_2_1", 
        "world/pinocchio/visuals/upperleg_sasm_2_2", 
        "world/pinocchio/visuals/upperleg_sasm_2_3", 
        "world/pinocchio/visuals/upperleg_sasm_2_4", 
        "world/pinocchio/visuals/upperleg_sasm_2_5", 
        "world/pinocchio/visuals/lowerleg_sasm_2_0", 
        "world/pinocchio/visuals/lowerleg_sasm_2_1", 
        "world/pinocchio/visuals/lowerleg_sasm_2_2", 
        "world/pinocchio/visuals/lowerleg_sasm_2_3", 
        "world/pinocchio/visuals/helbow_sasm_3_0", 
        "world/pinocchio/visuals/helbow_sasm_3_1", 
        "world/pinocchio/visuals/upperleg_sasm_3_0", 
        "world/pinocchio/visuals/upperleg_sasm_3_1", 
        "world/pinocchio/visuals/upperleg_sasm_3_2", 
        "world/pinocchio/visuals/upperleg_sasm_3_3", 
        "world/pinocchio/visuals/upperleg_sasm_3_4", 
        "world/pinocchio/visuals/upperleg_sasm_3_5", 
        "world/pinocchio/visuals/lowerleg_sasm_3_0", 
        "world/pinocchio/visuals/lowerleg_sasm_3_1", 
        "world/pinocchio/visuals/lowerleg_sasm_3_2", 
        "world/pinocchio/visuals/lowerleg_sasm_3_3", 
        "world/pinocchio/visuals/helbow_sasm_4_0", 
        "world/pinocchio/visuals/helbow_sasm_4_1", 
        "world/pinocchio/visuals/upperleg_sasm_4_0", 
        "world/pinocchio/visuals/upperleg_sasm_4_1", 
        "world/pinocchio/visuals/upperleg_sasm_4_2", 
        "world/pinocchio/visuals/upperleg_sasm_4_3", 
        "world/pinocchio/visuals/upperleg_sasm_4_4", 
        "world/pinocchio/visuals/upperleg_sasm_4_5", 
        "world/pinocchio/visuals/lowerleg_sasm_4_0", 
        "world/pinocchio/visuals/lowerleg_sasm_4_1", 
        "world/pinocchio/visuals/lowerleg_sasm_4_2", 
        "world/pinocchio/visuals/lowerleg_sasm_4_3", 
        "world/pinocchio/visuals/arm1_sasm_0", 
        "world/pinocchio/visuals/arm2_sasm_0", 
        "world/pinocchio/visuals/arm2_sasm_1", 
        "world/pinocchio/visuals/arm2_sasm_2", 
        "world/pinocchio/visuals/arm3_sasm_0", 
        "world/pinocchio/visuals/end_effector_sasm_0", 
        "world/pinocchio/visuals/end_effector_sasm_1", 
        "world/pinocchio/visuals/end_effector_sasm_2", 
        "world/pinocchio/visuals/gripper_0"
    ]

    node_list = []
    for node in node_name:
        node_list.append(node)

    if export_to_blender:
        project_path = os.path.abspath("blender/")

        python_file_path = project_path + "/pinToBlender.py"
        motion_file_path = project_path + "/motion.yaml"
        viz.viewer.gui.writeBlenderScript(python_file_path, node_list)
        viz.viewer.gui.setCaptureTransform(motion_file_path, node_list)

    # Tests
    if enable_viz == 1:
        viz.viewer.gui.addCurve("world/pinocchio/curve_1", my_state_machine.trajectory1.getAllPoint(dt), Color.lightBlue)
        viz.viewer.gui.addCurve("world/pinocchio/curve_2", my_state_machine.trajectory2.getAllPoint(dt), Color.lightYellow)
        viz.viewer.gui.addCurve("world/pinocchio/curve_3", my_state_machine.trajectory3.getAllPoint(dt), Color.lightRed)

    # main loop, updating the configuration vector q
    for i in range(int(duration / dt)):
        # start time for loop duration
        t0 = time.time()

        ### start controler

        q_current, dq_current, goal = my_state_machine.updateState(q_current, dq_current, i, add_goal_viz=visual_object)

        ### end controler

        # if i == 1565:
        #     print()

        if enable_viz:
            # to display the movement in a 3D viewport
            viz.display(q_current)  
            # viz.drawFrameVelocities(frame_id=sassa.model.getFrameId('framegripper'))
            # rot_matrix = np.eye(4)
            # rot_matrix[:3, :3] = pin.utils.rotate("z", np.pi)
            # pos = sassa.data.oMf[sassa.model.getFrameId('framegripper')].homogeneous # @ rot_matrix
            # viz.setCameraPose(pos)

        if export_to_blender and i%40 == 0:
            viz.viewer.gui.refresh ()
            viz.viewer.gui.captureTransform ()

        # log values
        log_com.append(pin.centerOfMass(sassa.model, sassa.data, q_current))
        log_goal.append(goal)
        IDX_Gripper = sassa.model.getFrameId('framegripper')
        frame_EF = [sassa.data.oMf[IDX_Gripper].homogeneous[:3, -1], \
            pin.getFrameVelocity(sassa.model, sassa.data, IDX_Gripper).vector[:3], np.array([0, 0, 0])]
        log_end_effector.append(frame_EF)

        # Add visual objects, CoM and trajectory
        if enable_viz == 1:
            CoM = list(sassa.com(q=q_current, v=dq_current))
            CoM = np.hstack([CoM[0], CoM[1], 1])
            CoM[2] = 0
            viz.viewer.gui.applyConfiguration("world/pinocchio/CoM", list(CoM))
        
        elif enable_viz == 2:
            com_projection.updatePlacement(q_current)
        
        # wait to have a real time sim
        if i % (1/dt) == 0 and enable_viz:
            # print the remaining time of the simulation in second
            print("time remaining :", duration-(i*dt))

        if enable_viz:
            tsleep = dt - (time.time() - t0)
            if tsleep > 0:
                # wait to have a consitente frame rate
                time.sleep(tsleep)

    if enable_viz == 1:
        viz.viewer.gui.deleteNode("world/pinocchio/curve_1", True)
        viz.viewer.gui.deleteNode("world/pinocchio/curve_2", True)
        viz.viewer.gui.deleteNode("world/pinocchio/curve_3", True)
        viz.viewer.gui.deleteNode("world/pinocchio/visuals/arm2_sasm_0", True)
        viz.viewer.gui.deleteNode("world/pinocchio/visuals/arm2_sasm_1", True)
        viz.viewer.gui.deleteNode("world/pinocchio/visuals/arm2_sasm_2", True)
        viz.viewer.gui.deleteNode("world/pinocchio/visuals/arm3_sasm_0", True)

    return log_com, log_goal, log_end_effector


if __name__ == "__main__":
    log_com, log_goal, log_end_effector = scenario1(robot_urdf_path="urdf/sassa-robot/robot.urdf", robot_file_path="urdf/sassa-robot/", enable_viz=1)

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