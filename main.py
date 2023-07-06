import time
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from init import initRobot, initViz
from controller import controller2IK, lookAt, useGripper, controller2IK2ndorder
from gripper import actuate_gripper
from neckController import neckController, check_joint_limit
from computeCollision import computeCollisions
from visualObject import CenterOfMass
from StateMachine import StateMahine
from trajectory import CircleTrajectory

sassa = initRobot("../sassa/robot_obj.urdf", "../sassa/")
viz = initViz(sassa, 2, add_ground=False, add_box=False)

duration = 40
dt = 0.01
realtime_sim = True

# q = np.zeros((25,))
q = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dq = np.zeros((24,))
d2q = np.zeros((24,))

init = True
is_open = False

# Object to show the projection on the ground of the center of masse 
com_projection = CenterOfMass(viz, sassa, "com")

# State machine to implement a scenario of action
my_state_machine = StateMahine()

# trajectory
my_trajectory = CircleTrajectory()
# origine x, y, z; raduis; omega
my_trajectory.circleTrajectoryXY(0.4, -0.05, 0.2, 2, 10)

for i in range(int(duration / dt)): # int(duration / dt)
    # start time for loop duration
    t0 = time.time()

    ### start controler

    # WORKING controller
    goal = my_trajectory.getPoint(i%360)
    q, dq = controller2IK2ndorder(q, dq, dt, sassa, i, viz, goal)
    # q, dq, _ = lookAt(q, dq, dt, sassa, i, viz, 1)
    # q, dq, _ = useGripper(q, dq, dt, sassa, i, viz)

    # TEST controller
    # ...

    # STATE MACHINE
    # update the current state of the robot
    # q, dq = my_state_machine.changeState(q, dq, dt, sassa, i, viz)


    # ACTUATE gripper
    """ q, is_gripper_end_actuated = actuate_gripper(sassa, q, dt, open=is_open)
    if is_gripper_end_actuated:
        is_open = not is_open
        is_gripper_end_actuated = False """

    init = False
    # q, dq = check_joint_limit(q, dq)
    viz.display(q)

    # Update Geometry models
    sassa.updateGeometryPlacements(q, visual=False)

    com_projection.updatePlacement(q)

    ### end controler

    # wait to have a real time sim
    if realtime_sim:
        tsleep = dt - (time.time() - t0)
        if tsleep > 0:
            time.sleep(tsleep)
            

