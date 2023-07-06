import time
import pinocchio as pin
import numpy as np
from numpy.linalg import inv,pinv,norm,eig,svd
import matplotlib.pyplot as plt
import sys
from pinocchio.visualize import MeshcatVisualizer
from pinocchio.robot_wrapper import RobotWrapper

def workSpace():
    """
    Draw a 3D shape that represent the reachable space of the end effector
    """
    start_time = time.time()

    model, collision_model, visual_model = pin.buildModelsFromUrdf("../sassa/robot.urdf", "../sassa/", pin.JointModelFreeFlyer())
    data  = model.createData()
    collision_data = collision_model.createData()

    q0 = pin.neutral(model)

    # 3D viewer

    realtime = False

    if realtime:
        viz = MeshcatVisualizer(model, collision_model, visual_model)
        try:
            viz.initViewer(open=True)
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install Python meshcat"
            )
            print(err)
            sys.exit(0)

        viz.loadViewerModel()
        viz.display(q0)
        viz.displayCollisions(False) # display the collision meshes
        viz.displayVisuals(True)

    dt = 0.1

    q = q0.copy()
    posX = [] # Log the value of the position of the end effector.
    posY = []
    posZ = []

    IDX_tool = model.getFrameId('OT')

    def Q1(t):
        upperLim = 90
        lowerLim = -90
        step = 180 / 100
        return np.deg2rad(lowerLim + step * t)

    def Q2(t):
        upperLim = 135
        lowerLim = -45
        step = 180 / 100
        return np.deg2rad(lowerLim + step * t)

    def Q3(t):
        upperLim = 120
        lowerLim = -120
        step = 240 / 100
        return np.deg2rad(lowerLim + step * t)

    for i in range(400): # 100 iterations per axis
        t0 = time.time()

        if i < 200:
            q[19] = np.random.uniform(np.deg2rad(-90), np.deg2rad(90))
            q[20] = np.random.uniform(np.deg2rad(-45), np.deg2rad(135))

            pin.framesForwardKinematics(model, data, q)
            pos = data.oMf[IDX_tool].translation
            posX.append(pos[0])
            posY.append(pos[1])
            posZ.append(pos[2])
        elif i < 250:
            q[19] = np.random.uniform(np.deg2rad(-90), np.deg2rad(90))
            q[20] = np.deg2rad(-45)
            q[21] = np.random.uniform(np.deg2rad(-120), np.deg2rad(0))
            pin.framesForwardKinematics(model, data, q)
            pos = data.oMf[IDX_tool].translation
            posX.append(pos[0])
            posY.append(pos[1])
            posZ.append(pos[2])
        elif i < 400:
            q[19] = np.random.uniform(np.deg2rad(-90), np.deg2rad(90))
            q[20] = np.deg2rad(135)
            q[21] = np.random.uniform(np.deg2rad(0), np.deg2rad(120))
            pin.framesForwardKinematics(model, data, q)
            pos = data.oMf[IDX_tool].translation
            posX.append(pos[0])
            posY.append(pos[1])
            posZ.append(pos[2])


        # real time viz
        if realtime:
            viz.display(q)
            tsleep = dt - (time.time() - t0)
            if tsleep > 0:
                time.sleep(tsleep)


    """ plt.figure()
    plt.scatter(posY, posZ)
    plt.legend(["reachable position"], loc ="lower right")
    plt.title("Work space on X axis")
    plt.show()

    plt.figure()
    plt.scatter(posX, posZ)
    plt.legend(["reachable position"], loc ="lower right")
    plt.title("Work space on Y axis")
    plt.show() """


    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(0.256, 0.0, 0.098, color='r')
    ax.scatter(posX, posY, posZ)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()



if __name__ == "__main__":
    workSpace()