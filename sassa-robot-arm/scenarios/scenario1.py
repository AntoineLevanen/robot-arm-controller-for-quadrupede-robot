import time
import numpy as np
import matplotlib.pyplot as plt

import sys
import os
path = os.path.abspath("sassa-robot-arm")
sys.path.append(path)
from init import initRobot, initViz
from visualObject import CenterOfMass

from scenario1StateMachine import StateMahineScenario1

sassa = initRobot("urdf/sassa-robot/robot.urdf", "urdf/sassa-robot/")
# sassa = initRobot("urdf/sassa-robot-short-arm/robot.urdf", "urdf/sassa-robot-short-arm/")
viz = initViz(sassa, 2, add_ground=True, add_box=True, box_config=[0.4, 0.0, 0.04])

duration = 60 # vizualization duration
dt = 0.05 # delta time
trajectory_step = int(duration / dt)
realtime_viz = True # flag to enable "realtime" play

# robot start configuration, velocity and acceleration
q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0])

q_current = q0_ref.copy()

dq_current = np.zeros((sassa.model.nv,))
d2q_current = np.zeros((sassa.model.nv,))

# Object to show the projection on the ground of the center of masse 
com_projection = CenterOfMass(viz, sassa, "com")

my_state_machine = StateMahineScenario1(sassa, viz, dt, q0_ref, curve_resolution=500)

init = True

# main loop, updating the configuration vector q
for i in range(int(duration / dt)): # int(duration / dt)
    # start time for loop duration
    t0 = time.time()

    ### start controler

    # Implement the scenario here (State Machine, ...)
    q_current, dq_current = my_state_machine.updateState(q_current, dq_current, i)

    ### end controler

    init = False

    viz.display(q_current)

    # wait to have a real time sim
    # if i % (1/dt) == 0:
    #     print("time remaining :", duration-(i*dt))

    if realtime_viz:
        tsleep = dt - (time.time() - t0)
        if tsleep > 0:
            # wait to have a consitente frame rate
            time.sleep(tsleep)

"""
Plot different values...
pose of center of masse with different arm length...
error between End Effector target position and the real one...
"""