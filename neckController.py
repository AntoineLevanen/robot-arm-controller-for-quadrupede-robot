import time
import sys
import pinocchio as pin
import numpy as np
from numpy.linalg import inv,pinv,norm,eig,svd
from numpy import sin, cos
from computeCollision import computeCollisions
from visualObject import SphereGoal

   

def check_joint_limit(q, dq):
    from numpy import deg2rad as d2r
    # if a limit is reach
    flag = False

    # limit for the legs
    for i in range(4):
        if q[7 + 3 * i] < d2r(-45):
            dq[6 + 3 * i] = 0
            q[7 + 3 * i] = d2r(-45)
            flag = True
        if q[7 + 3 * i] > d2r(45):
            dq[6 + 3 * i] = 0
            q[7 + 3 * i] = d2r(45)
            flag = True

        if q[8 + 3 * i] < d2r(-90):
            dq[7 + 3 * i] = 0
            q[8 + 3 * i] = d2r(-90)
            flag = True
        if q[8 + 3 * i] > d2r(90):
            dq[7 + 3 * i] = 0
            q[8 + 3 * i] = d2r(90)
            flag = True

        if q[9 + 3 * i] < d2r(-135):
            dq[8 + 3 * i] = 0
            q[9 + 3 * i] = d2r(-135)
            flag = True
        if q[9 + 3 * i] > d2r(135):
            dq[8 + 3 * i] = 0
            q[9 + 3 * i] = d2r(135)
            flag = True

    # limit for the neck
    if q[19] < d2r(-90):
        dq[18] = 0
        q[19] = d2r(-90)
        flag = True
    if q[19] > d2r(90):
        dq[18] = 0
        q[19] = d2r(90)
        flag = True

    if q[20] < d2r(-45):
        dq[19] = 0
        q[20] = d2r(-45)
        flag = True
    if q[20] > d2r(135):
        dq[19] = 0
        q[20] = d2r(135)
        flag = True

    if q[21] < d2r(-120):
        dq[20] = 0
        q[21] = d2r(-120)
        flag = True
    if q[21] > d2r(120):
        dq[20] = 0
        q[21] = d2r(120)
        flag = True

    if q[22] < d2r(-180):
        dq[21] = 0
        q[22] = d2r(-120)
        flag = True
    if q[22] > d2r(180):
        dq[21] = 0
        q[22] = d2r(-120)
        flag = True

    return q, dq

def check_joint_speed(dq):
    flag = False
    # limit for the neck
    if dq[18] < -2:
        dq[18] = -2.0
        flag = True
    if dq[18] > 2:
        dq[18] = 2.0
        flag = True

    if dq[19] < -2:
        dq[19] = -2.0
        flag = True
    if dq[19] > 2:
        dq[19] = 2.0
        flag = True

    if dq[20] < -2:
        dq[20] = -2.0
        flag = True
    if dq[20] > 2:
        dq[20] = 2.0
        flag = True

    if dq[21] < -2:
        dq[21] = -2.0
        flag = True
    if dq[21] > 2:
        dq[21] = 2.0
        flag = True

    if dq[22] < -2:
        dq[22] = -2.0
        flag = True
    if dq[22] > 2:
        dq[22] = 2.0
        flag = True

    if dq[23] < -2:
        dq[23] = -2.0
        flag = True
    if dq[23] > 2:
        dq[23] = 2.0
        flag = True
    return dq

""" def neckController(q, dq, dt, robot, i, viz):
    e1 = 10e-3

    model = robot.model
    data = robot.data
    
    # compute the forward kinematics from the vector q0
    pin.framesForwardKinematics(model, data, robot.q0)

    # Get the robot frames
    IDX_Base = model.getFrameId('body_sasm')
    IDX_tool = model.getFrameId('OT')

    # define the target frame of the base and the end effector
    oMgoalBase = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.0]))
    
    oMgoalTool = pin.SE3(np.eye(3), np.array([0.45, 0.0, 0.]))

    # add a visual sphere to represent the tool target (replace w/ a frame for orientation)
    sphereGoal(viz, oMgoalTool.translation)

    # make sure the point is reachable by the end effector

    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(model,data,q)
    pin.computeJointJacobians(model,data,q)

    # base task
    oMbase = data.oMf[IDX_Base]
    base_nu = pin.log(oMbase.inverse() * oMgoalBase).vector
    o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)# [:3,:]
    o_BaseFixed = oMbase.translation - oMgoalBase.translation

    # End effector task
    oMtool = data.oMf[IDX_tool]
    o_Jtool3 = pin.computeFrameJacobian(model,data,q,IDX_tool, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Tool = oMtool.translation - oMgoalTool.translation
    
    vq = 10 * np.ones(robot.model.nv)
    # Tasks by order of priority
    # vq += pinv(o_Jbase) @ base_nu
    vq = pinv(o_Jbase) @ base_nu # 1er
    Pbase = np.eye(24) - pinv(o_Jbase) @ o_Jbase
    vq += pinv(o_Jtool3 @ Pbase) @ (-o_Tool - o_Jtool3 @ vq) # 2eme

    # managing joint velocity limits
    vq = check_joint_speed(vq)

    # compute the next configuration and display it
    q = pin.integrate(model, q, vq * dt)

    # managing joint angle limits
    q, vq = check_joint_limit(q, vq)

    return q, vq """


# Initialization of the controller's parameters
q_ref = np.zeros(25)
dq_ref = np.zeros(24)
d2q_ref = np.zeros(24)
d2qa_ref = np.zeros(18)

def neckController(q, dq, dt, robot, i, viz):

    qa = q[7:] # actuated joint
    dqa = dq[6:] 

    qa_ref = np.zeros(robot.model.nv - 6)
    dqa_ref = np.zeros(robot.model.nv - 6)

    global d2qa_ref, d2q_ref, dq_ref, q_ref

    e1 = 10e-3

    model = robot.model
    data = robot.data

    # Get the robot frames
    IDX_Base = model.getFrameId('body_sasm')
    IDX_tool = model.getFrameId('OT')

    # define the target frame of the base and the end effector
    oMgoalBase = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.0]))
    
    oMgoalTool = pin.SE3(np.eye(3), np.array([0.45, 0.0, 0.0]))

    # add a visual sphere to represent the tool target (replace w/ a frame for orientation)
    SphereGoal(viz, oMgoalTool.translation, "goal1")

    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(model,data,q)
    pin.computeJointJacobians(model,data,q)

    # base task
    oMbase = data.oMf[IDX_Base]
    base_nu = pin.log(oMbase.inverse() * oMgoalBase).vector
    o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)# [:3,:]
    o_BaseFixed = oMbase.translation - oMgoalBase.translation

    # End effector task
    oMtool = data.oMf[IDX_tool]
    o_Jtool3 = pin.computeFrameJacobian(model,data,q,IDX_tool, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Tool = oMtool.translation - oMgoalTool.translation
    
    # computing mass matrix and dynamic drift
    M = pin.crba(robot.model, robot.data, q_ref)
    b = pin.rnea(robot.model, robot.data, q_ref, dq_ref, d2q_ref)

    # computing torques
    Kp_tau = 100
    Kd_tau = 2 * np.sqrt(Kp_tau) # 0.2
    torques_ref = (-Kp_tau * (q_ref[7:] - q[7:])) - (Kd_tau * dqa_ref)

    # computing joint accelerations
    d2qa_ref = inv(M[6:, 6:]) @ (torques_ref - b[6:])
    d2q_ref = np.hstack((np.zeros(6), d2qa_ref))

    # computing joint velocity
    dqa_ref += d2qa_ref * dt
    dq_ref = np.hstack((np.zeros(6), dqa_ref))

    # compute the next configuration and display it
    q_ref = pin.integrate(model, q_ref, dq_ref * dt)
    qa_ref = q_ref[7:]

    # managing joint angle limits
    vq = check_joint_speed(dq_ref)
    q, vq = check_joint_limit(q_ref, dq_ref)

    # Implement collision checking

    return q, vq