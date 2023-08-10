import time
import numpy as np
import os
import pinocchio as pin
import matplotlib.pyplot as plt
from init import initRobot, initViz
from controller import controllerCLIK2ndorder
from gripper import actuate_gripper
from visualObject import CenterOfMass
from trajectory import CircleTrajectory
import gepetto.corbaserver as gui

urdf_path = os.path.abspath("urdf/sassa-robot-short-arm/robot.urdf")
file_path = os.path.abspath("urdf/sassa-robot-short-arm/")
sassa = initRobot(urdf_path, file_path)
viz = initViz(sassa, 1, add_ground=False, add_box=False)

duration = 30 # vizualization duration
dt = 0.04 # delta time
trajectory_step = int(duration / dt)
realtime_viz = True
export_to_blender = False

q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])

q_current = q0_ref.copy()

dq_current = np.zeros((sassa.model.nv,))
d2q_current = np.zeros((sassa.model.nv,))

init = True
is_close = "open"

# Object to show the projection on the ground of the center of masse 
# com_projection = CenterOfMass(viz, sassa, "com")

# circular trajectory
my_trajectory = CircleTrajectory()
# origine x, y, z, raduis, omega
my_trajectory.circleTrajectoryXY(0.35, -0.01, 0.3, 0.02, 2)

err = [[0, 0, 0]]

# to export to blender

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

    python_file_path = project_path + "pinToBlender.py"
    motion_file_path = project_path + "motion.yaml"
    viz.viewer.gui.writeBlenderScript(python_file_path, node_list)
    viz.viewer.gui.setCaptureTransform(motion_file_path, node_list)

# main loop, updating the configuration vector q
for i in range(int(duration / dt)):
    # start time for loop duration
    t0 = time.time()

    ### start controler

    # WORKING controller
    goal = my_trajectory.getPoint(i%360) # circular trajectory
    q_current, dq_current, _ = controllerCLIK2ndorder(q_current, dq_current, dt, sassa, \
                                            init, viz, q0_ref, goal, add_goal_sphere=False)

    if export_to_blender:
        viz.viewer.gui.refresh ()
        viz.viewer.gui.captureTransform ()


    # ACTUATE gripper
    # q_current, is_gripper_end_actuated = actuate_gripper(sassa, q_current, dt, action="close")
    # if is_gripper_end_actuated:
    #     is_close = "close" if is_close == "open" else "open"
    #     is_gripper_end_actuated = False

    init = False
    viz.display(q_current)

    ### end controler

    # wait to have a real time sim
    if realtime_viz:
        tsleep = dt - (time.time() - t0)
        if tsleep > 0:
            time.sleep(tsleep)
