import time
import numpy as np
import matplotlib.pyplot as plt
from init import initRobot, initViz

sassa = initRobot("urdf/sassa-robot/robot_obj.urdf", "urdf/sassa-robot/")
viz = initViz(sassa, 2, add_ground=True, add_box=False)

duration = 60 # vizualization duration
dt = 0.05 # delta time
trajectory_step = int(duration / dt)
realtime_viz = True # flag to enable "realtime" play

# robot start configuration, velocity and acceleration
q0_ref = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, np.pi/8, -np.pi/4, 0.0, 0.0, 0.0])

q_current = q0_ref.copy()

dq_current = np.zeros((24,))
d2q_current = np.zeros((24,))

# Object to show the projection on the ground of the center of masse 
com_projection = CenterOfMass(viz, sassa, "com")



# main loop, updating the configuration vector q
for i in range(int(duration / dt)): # int(duration / dt)
    # start time for loop duration
    t0 = time.time()

    ### start controler

    # Implement the scenario here (State Machine, ...)

    ### end controler

    # wait to have a real time sim
    if i % (1/dt) == 0:
        print("time remaining :", duration-(i*dt))

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