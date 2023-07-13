import time
import math
import sys
import pinocchio as pin
import numpy as np
from numpy.linalg import inv,pinv,norm,eig,svd
from numpy import sin, cos
from computeCollision import computeCollisions
from visualObject import SphereGoal


def controller(q, dq, dt, robot, init, i, viz):
    """
    Go from one point to another. Using CLIK (Closed Loop Inverse Kinematics)
    q : current configuration of the robot
    dq : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    init : flag to init variable
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    return : robot configuration, robot velocity
    """

    global flag, flag2, flag3, oMgoalBase, oMgoalTool
    if init:
        flag = True
        flag2 = False

    e1 = 0.015

    model = robot.model
    data = robot.data
    # Get the robot frames
    IDX_Base = model.getFrameId('body_sasm')
    IDX_tool = model.getFrameId('OT')
    IDX_FLfoot = model.getFrameId('FL_foot_frame')
    IDX_FRfoot = model.getFrameId('FR_foot_frame')
    IDX_HLfoot = model.getFrameId('HL_foot_frame')
    IDX_HRfoot = model.getFrameId('HR_foot_frame')

    # define the height of the feet in world Z axis
    feetHeight = 0 # -0.340

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feetHeight]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feetHeight]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feetHeight]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feetHeight]))

    # compute the forward kinematics from the vector q0
    pin.framesForwardKinematics(model, data, robot.q0)

    # define the target frame of the base and the end effector
    
    if init:
        oMgoalBase = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.0]))
        
        oMgoalTool = pin.SE3(np.eye(3), np.array([0.5, 0.0, 0.4 + feetHeight]))
        SphereGoal(viz, oMgoalTool.translation, "goal1")

    # make sure the point is reachable by the end effector
    """ distX = oMgoalTool.translation[0] - 0.256
    distY = 0
    distZ = oMgoalTool.translation[2] - 0.098
    distXZ = np.sqrt(distX**2 + distY**2)

    if distXZ > 0.240:
        print("Attention point non atteignable!")
        sys.exit(2) """

    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(model,data,q)
    pin.computeJointJacobians(model,data,q)

    # compute feet position error and Jacobian
    oMflfoot = data.oMf[IDX_FLfoot]
    o_Jflfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_flfoot = oMflfoot.translation - oMflfootGoal.translation

    oMfrfoot = data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfoot.translation - oMfrfootGoal.translation

    oMhlfoot = data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfoot.translation - oMhlfootGoal.translation

    oMhrfoot = data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfoot.translation - oMhrfootGoal.translation

    # base task
    oMbase = data.oMf[IDX_Base]
    base_nu = pin.log(oMbase.inverse() * oMgoalBase).vector
    o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)# [:3,:]
    o_BaseFixed = oMbase.translation - oMgoalBase.translation

    # End effector task
    oMtool = data.oMf[IDX_tool]
    o_Jtool3 = pin.computeFrameJacobian(model,data,q,IDX_tool, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_TG = oMtool.translation - oMgoalTool.translation
    

    # Tasks by order of priority
    vq = -pinv(o_Jflfoot3) @ o_flfoot # 1er
    Pflfoot = np.eye(24) - pinv(o_Jflfoot3) @ o_Jflfoot3
    vq += pinv(o_Jfrfoot3 @ Pflfoot) @ (-o_frfoot - o_Jfrfoot3 @ vq) # 2eme

    Pfrfoot = Pflfoot - pinv(o_Jfrfoot3 @ Pflfoot) @ o_Jfrfoot3 @ Pflfoot
    vq += pinv(o_Jhlfoot3 @ Pfrfoot) @ (-o_hlfoot - o_Jhlfoot3 @ vq) # 3eme

    Phlfoot = Pfrfoot - pinv(o_Jhlfoot3 @ Pfrfoot) @ o_Jhlfoot3 @ Pfrfoot
    vq += pinv(o_Jhrfoot3 @ Phlfoot) @ (-o_hrfoot - o_Jhrfoot3 @ vq) # 4eme

    Phrfoot = Phlfoot - pinv(o_Jhrfoot3 @ Phlfoot) @ o_Jhrfoot3 @ Phlfoot
    vq += pinv(o_Jtool3 @ Phrfoot) @ (-o_TG - o_Jtool3 @ vq) # 5eme (tool)

    # vq += pinv(o_Jbase) @ base_nu

    """ Ptool = Phrfoot - pinv(o_Jtool3 @ Phrfoot) @ o_Jtool3 @ Phrfoot
    vq += pinv(o_Jbase @ Ptool) @ (-o_BaseFixed - o_Jbase @ vq) """

    # compute the next configuration and display it
    q = pin.integrate(model, q, vq * dt)

    if norm(o_TG) < e1:
        if flag2:
            oMgoalTool = pin.SE3(np.eye(3), np.array([0.5, 0.2, 0.3 + feetHeight]))
            flag2 = False
        else:
            oMgoalTool = pin.SE3(np.eye(3), np.array([0.5, -0.2, 0.3 + feetHeight]))
            flag2 = True

    return q, vq

def controller2IK(q, dq, dt, robot, i, viz, goal):
    """
    The gripper follow the desired trajectory using first order inverse kinematics (IK)
    q : current configuration of the robot
    dq : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    init : flag to init variable
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    goal : goal position, velocity and acceleration of the end effector
    return : robot configuration, robot velocity
    """

    q0_ref = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Get the robot frames
    IDX_Camera = robot.model.getFrameId('framecamera')
    IDX_Gripper = robot.model.getFrameId('framegripper')
    IDX_Base = robot.model.getFrameId('body_sasm')
    IDX_FLfoot = robot.model.getFrameId('FL_foot_frame')
    IDX_FRfoot = robot.model.getFrameId('FR_foot_frame')
    IDX_HLfoot = robot.model.getFrameId('HL_foot_frame')
    IDX_HRfoot = robot.model.getFrameId('HR_foot_frame')

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, 0]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, 0]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, 0]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, 0]))

    # define the target frame of the end effector
    
    goal_Gripper_position = goal[0]
    goal_Gripper_velocity = goal[1]
    goal_Gripper_acceleration = goal[2]

    oMgoalGripper = pin.SE3(np.eye(3), np.array([goal_Gripper_position[0], goal_Gripper_position[1], goal_Gripper_position[2]]))

    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(robot.model,robot.data,q)
    pin.computeJointJacobians(robot.model,robot.data,q)

    # compute feet position error and Jacobian
    oMflfoot = robot.data.oMf[IDX_FLfoot]
    o_Jflfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_flfoot = oMflfoot.translation - oMflfootGoal.translation

    oMfrfoot = robot.data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfoot.translation - oMfrfootGoal.translation

    oMhlfoot = robot.data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfoot.translation - oMhlfootGoal.translation

    oMhrfoot = robot.data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfoot.translation - oMhrfootGoal.translation

    # Gripper task
    oMGripper = robot.data.oMf[IDX_Gripper]
    tool_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    o_JGripper = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Gripper = oMGripper.translation - oMgoalGripper.translation

    nu_feet = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    J_feet = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])

    # Tasks by order of priority
    K1 = 1
    vq = -K1 * pinv(J_feet) @ nu_feet # first task, fixe the feet
    
    Pfeet = np.eye(24) - pinv(J_feet) @ J_feet
    vq += pinv(o_JGripper @ Pfeet) @ (-o_Gripper - o_JGripper @ vq) # second task, move the gripper

    PGripper = Pfeet - pinv(o_JGripper @ Pfeet) @ o_JGripper @ Pfeet
    q_temp = q0_ref - q
    q_temp = np.hstack([q_temp[:3], QUATTOXYZ(q_temp[3:7]), q_temp[7:]])
    J_posture = np.eye(24)

    vq += J_posture @ q_temp # third task, regulation task

    # compute the next configuration and display it
    q = pin.integrate(robot.model, q, vq * dt)

    return q, vq

def controllerCLIK2ndorderPositionOnly(q_current, dq_current, dt, robot, init, viz, goal, q0_ref):
    """
    The gripper follow the desired trajectory using second order inverse kinematics with close loop (CLIK)
    Controlling in Position only
    q_current : current configuration of the robot
    dq_current : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    init : flag to init variable
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    goal : goal position, velocity and acceleration of the end effector
    return : robot configuration, robot velocity
    """

    global sphere_goal

    # Get the robot frames
    IDX_Base = robot.model.getFrameId('body_sasm')
    IDX_Gripper = robot.model.getFrameId('framegripper')
    IDX_FLfoot = robot.model.getFrameId('FL_foot_frame')
    IDX_FRfoot = robot.model.getFrameId('FR_foot_frame')
    IDX_HLfoot = robot.model.getFrameId('HL_foot_frame')
    IDX_HRfoot = robot.model.getFrameId('HR_foot_frame')

    feet_height = 0.016

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feet_height]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feet_height]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feet_height]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feet_height]))

    # define the target frame of the end effector
    
    goal_Gripper_position = goal[0]
    goal_Gripper_velocity = goal[1]
    goal_Gripper_acceleration = goal[2]

    # target position of the end effector
    oMgoalGripper = pin.SE3(np.eye(3), np.array([goal_Gripper_position[0], goal_Gripper_position[1], goal_Gripper_position[2]]))

    if init:
        sphere_goal = SphereGoal(viz, oMgoalGripper.translation, "goal1")
    sphere_goal.moveGoalVisual(oMgoalGripper.translation)

    # second order Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    robot.forwardKinematics(q_current, v=dq_current, a=0 * dq_current)
    pin.computeJointJacobians(robot.model,robot.data,q_current)

    # compute feet position error and Jacobian
    oMflfoot = robot.data.oMf[IDX_FLfoot] # get placement from world frame o to frame f
    o_Jflfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:] # take only linear velocity
    o_flfoot = oMflfootGoal.translation - oMflfoot.translation # desired velocity of the foot frame (position error)
    err_vel_fl_foot = o_flfoot - (o_Jflfoot3 @ dq_current) # velocity error

    oMfrfoot = robot.data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfootGoal.translation - oMfrfoot.translation
    err_vel_fr_foot = o_frfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3] # two posibility to compute the current frame velocity

    oMhlfoot = robot.data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfootGoal.translation - oMhlfoot.translation
    err_vel_hl_foot = o_hlfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    oMhrfoot = robot.data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfootGoal.translation - oMhrfoot.translation
    err_vel_hr_foot = o_hrfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    # Gripper jacobian and error
    oMGripper = robot.data.oMf[IDX_Gripper]

    # gripper_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    o_JGripper = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Gripper = oMgoalGripper.translation - oMGripper.translation
    err_vel_gripper = o_Gripper - (o_JGripper @ dq_current)
    
    oMgoal_gripper = pin.SE3(np.eye(3), np.array(goal_Gripper_position))

    # Base jacobian and error
    oMBase = robot.data.oMf[IDX_Base]

    oMgoal_base = pin.SE3(np.eye(3), np.array([0, 0, 0.4]))
    base_nu = pin.log(oMBase.inverse() * oMgoal_base).vector

    o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)
    err_vel_base = np.array([0, 0, 0, 0, 0, 0]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector


    # Stack the different terme in vectors to have on task for all four feet
    e = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    e_dot = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot])
    J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])
    x_ddot = np.hstack([np.zeros(12)])

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot)
    a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    # Stack the current acceleration of each feet frame
    J_dot_q_dot = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])

    # gains
    K1 = 300
    K2 = 2*np.sqrt(K1)

    # Tasks in order of priority
    # It is posible to scale task to affect the extremum of the velocity and acceleration, scale factor [0;1]
    # first task with higher priority, fixe the feet on the ground 
    d2q = pinv(J) @ (x_ddot - J_dot_q_dot + K2 * e_dot + K1 * e) 

    # Null Space of the first task
    P0 = np.eye(robot.model.nv) - pinv(J) @ J
    # second task with less priority, move the gripper
    d2q += pinv(o_JGripper @ P0) @ (goal_Gripper_acceleration - a_gripper + K2 * err_vel_gripper + K1 * np.array(o_Gripper))

    P1 = P0 - pinv(o_JGripper @ P0) @ o_JGripper @ P0
    d2q += pinv(o_JBase @ P1) @ (np.array([0, 0, 0, 0, 0, 0]) - a_base + K2 * err_vel_base + K1 * base_nu)

    # Add a Regulation task to fill the free remaining dof
    # computing the error in position in the configuration space base : xyz,abc
    q_temp = q0_ref - q_current
    q_temp = np.hstack([[0, 0, 0, 0, 0, 0], q_temp[7:]])

    K3 = 1
    J_posture = np.eye(robot.model.nv)
    J_posture[:6, :6] = 0
    # d2q += K3 * J_posture @ q_temp

    # compute the velocity
    dq_next = d2q * dt

    # compute the next configuration
    q_next = pin.integrate(robot.model, q_current, dq_next * dt)

    return q_next, dq_next

def controllerCLIK2ndorder(q_current, dq_current, dt, robot, init, viz, goal, q0_ref):
    """
    The gripper follow the desired trajectory using second order closed loop inverse kinematics (CLIK)
    Controlling Position and Orientation of the end effector with the four leg stick to the ground, 
    the body will adapte to follow the end effector movement
    q_current : current configuration of the robot
    dq_current : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    init : flag to init variable
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    goal : goal position, velocity and acceleration of the end effector
    return : robot configuration, robot velocity
    """
    
    global sphere_goal

    # Get the robot frames
    IDX_Base = robot.model.getFrameId('body_sasm')
    IDX_Gripper = robot.model.getFrameId('framegripper')
    IDX_FLfoot = robot.model.getFrameId('FL_foot_frame')
    IDX_FRfoot = robot.model.getFrameId('FR_foot_frame')
    IDX_HLfoot = robot.model.getFrameId('HL_foot_frame')
    IDX_HRfoot = robot.model.getFrameId('HR_foot_frame')

    feet_height = 0.016

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feet_height]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feet_height]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feet_height]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feet_height]))

    # define the target frame of the end effector
    
    goal_Gripper_position = goal[0]
    goal_Gripper_velocity = goal[1]
    goal_Gripper_acceleration = goal[2]

    # target position of the end effector
    oMgoalGripper = pin.SE3(np.eye(3), np.array([goal_Gripper_position[0], goal_Gripper_position[1], goal_Gripper_position[2]]))

    if init:
        sphere_goal = SphereGoal(viz, oMgoalGripper.translation, "goal1")
    sphere_goal.moveGoalVisual(oMgoalGripper.translation)

    # second order Closed Loop Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    robot.forwardKinematics(q_current, v=dq_current, a=0 * dq_current)
    pin.computeJointJacobians(robot.model,robot.data,q_current)

    # compute feet position error and Jacobian
    oMflfoot = robot.data.oMf[IDX_FLfoot] # get placement from world frame o to frame f
    o_Jflfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:] # take only linear velocity
    o_flfoot = oMflfootGoal.translation - oMflfoot.translation # desired velocity of the foot frame (position error)
    err_vel_fl_foot = o_flfoot - (o_Jflfoot3 @ dq_current) # velocity error

    oMfrfoot = robot.data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfootGoal.translation - oMfrfoot.translation
    err_vel_fr_foot = o_frfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3] # two posibility to compute the current frame velocity

    oMhlfoot = robot.data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfootGoal.translation - oMhlfoot.translation
    err_vel_hl_foot = o_hlfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    oMhrfoot = robot.data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfootGoal.translation - oMhrfoot.translation
    err_vel_hr_foot = o_hrfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    # Gripper jacobian and error
    oMGripper = robot.data.oMf[IDX_Gripper]
    o_JGripper = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)
    # get the jacobian matrix of translation part and on orientation (y axis)
    o_JGripper = np.vstack([o_JGripper[:3], o_JGripper[4]])
    o_Gripper = oMgoalGripper.translation - oMGripper.translation
    err_vel_gripper = np.hstack([o_Gripper, [0, 0, 0]]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    err_vel_gripper = np.hstack([err_vel_gripper[:3], err_vel_gripper[4]])

    # define a rotation for the end effector
    oMgoalGripper.rotation = pin.utils.rotate('y', np.pi/4)
    gripper_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    # error vector position + one orientation
    gripper_nu = np.hstack([gripper_nu[:3], gripper_nu[4]])

    # Base jacobian and error
    oMBase = robot.data.oMf[IDX_Base]
    oMgoal_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
    base_nu = pin.log(oMBase.inverse() * oMgoal_base).vector
    o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)
    err_vel_base = np.array([0, 0, 0, 0, 0, 0]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector


    # Stack the different terme in vectors to have on task for all four feet
    e = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    e_dot = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot])
    J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])
    x_ddot = np.hstack([np.zeros(12)])

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot)
    a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np
    a_gripper = np.hstack([a_gripper[:3], a_gripper[4]])
    a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np

    # Stack the current acceleration of each feet frame
    J_dot_q_dot = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])

    # gains
    # tune the gain to compensate the overshoot
    K1 = 20
    K2 = 2*np.sqrt(K1)

    # Tasks in order of priority
    # It is posible to scale task to affect the extremum of the velocity and acceleration, scale factor [0;1]
    # first task with higher priority, fixe the feet on the ground 
    d2q = pinv(J) @ (x_ddot - J_dot_q_dot + K2 * e_dot + K1 * e) 

    # Null Space of the first task
    P0 = np.eye(robot.model.nv) - pinv(J) @ J
    # second task with less priority, move the gripper
    d2q += pinv(o_JGripper @ P0) @ (np.hstack([goal_Gripper_acceleration, [0]]) - a_gripper + K2 * err_vel_gripper + K1 * gripper_nu)

    P1 = P0 - pinv(o_JGripper @ P0) @ o_JGripper @ P0
    # constrain the CoM position in the center of the support polygone, only in X and Y
    d2q += pinv(o_JBase[:2,:] @ P1) @ (np.array([0, 0]) - a_base[:2] + K2 * err_vel_base[:2] + K1 * base_nu[:2])

    # Add a Regulation Task to fill the free remaining dof
    # computing the error in position in the configuration space base : xyz,abc
    q_temp = q0_ref - q_current
    q_temp = np.hstack([[0, 0, 0, 0, 0, 0], q_temp[7:]])

    K3 = 1
    J_posture = np.eye(robot.model.nv)
    J_posture[:6, :6] = 0
    d2q += K3 * J_posture @ q_temp

    # compute the velocity
    dq_next = d2q * dt

    # compute the next configuration
    q_next = pin.integrate(robot.model, q_current, dq_next * dt)

    return q_next, dq_next

def QUATTOXYZ(q1):
    """
    Convert a Quaternion in angle axis 
    q1 : Normalized quaternion
    return : axis angle
    """
    x, y, z = 0, 0, 0
    if (q1[3] > 1):
        q1 = pin.Quaternion(qi).normalize
    angle = 2 * np.arccos(q1[3])
    s = np.sqrt(1-q1[3]*q1[3]) # assuming quaternion normalised then w is less than 1, so term always positive.
    if (s < 0.001): # test to avoid divide by zero, s is always positive due to sqrt
        # if s close to zero then direction of axis not important
        x = q1[0] # if it is important that axis is normalised then replace with x=1; y=z=0;
        y = q1[1]
        z = q1[2]
    else:
        x = q1[0] / s # normalise axis
        y = q1[1] / s
        z = q1[2] / s
    
    return np.array([x, y, z])

def lookAt(q, dq, dt, robot, i, viz, configuration, goal=None):
    """
    Take a look at something, invsetigation a point pf interest
    q : current configuration of the robot
    dq : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    configuration : Pre-define configuration: choose 1, 2 or 3
    goal : Pinocchio::SE3 object that indicate the target
    return : robot configuration, robot velocity, flag to know wether te action is finish or not 
    """
    global oMgoalBase, oMgoalGripper, oMgoalCamera

    e1 = 0.015

    model = robot.model
    data = robot.data
    # Get the robot frames
    IDX_Camera = model.getFrameId('framecamera')
    IDX_Base = model.getFrameId('body_sasm')
    IDX_FLfoot = model.getFrameId('FL_foot_frame')
    IDX_FRfoot = model.getFrameId('FR_foot_frame')
    IDX_HLfoot = model.getFrameId('HL_foot_frame')
    IDX_HRfoot = model.getFrameId('HR_foot_frame')

    # define the height of the feet in world Z axis
    feetHeight = 0 # -0.340

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feetHeight]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feetHeight]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feetHeight]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feetHeight]))

    # define the target frame of the base and the end effector
    if goal is None:
        if configuration == 1:
            oMgoalBase = pin.SE3(np.eye(3), np.array([0.1, 0.0, 0.3 + feetHeight]))
            oMgoalCamera = pin.SE3(np.eye(3), np.array([0.45, 0.1, 0.02 + feetHeight])) # look behind the table
        if configuration == 2:
            oMgoalBase = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.35 + feetHeight]))
            oMgoalCamera = pin.SE3(np.eye(3), np.array([0.25, .14, feetHeight])) # look at the FLFoot
        if configuration == 3:
            oMgoalBase = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.35 + feetHeight]))
            oMgoalCamera = pin.SE3(np.eye(3), np.array([0.4, 0.0, 0.2 + feetHeight])) # look at table top

        SphereGoal(viz, oMgoalCamera.translation, "GoalCamera")

    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(model,data,q)
    pin.computeJointJacobians(model,data,q)

    # compute feet position error and Jacobian
    oMflfoot = data.oMf[IDX_FLfoot]
    o_Jflfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_flfoot = oMflfoot.translation - oMflfootGoal.translation

    oMfrfoot = data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfoot.translation - oMfrfootGoal.translation

    oMhlfoot = data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfoot.translation - oMhlfootGoal.translation

    oMhrfoot = data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfoot.translation - oMhrfootGoal.translation

    # base task
    oMbase = data.oMf[IDX_Base]
    o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    base_nu = pin.log(oMbase.inverse() * oMgoalBase).vector
    o_BaseFixed = oMbase.translation - oMgoalBase.translation
    if configuration == 1:
        o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:2,:]
        o_BaseFixed = oMbase.translation[:2] - oMgoalBase.translation[:2]

    # Camera task
    oMCamera = data.oMf[IDX_Camera]
    tool_nu = pin.log(oMCamera.inverse() * oMgoalCamera).vector
    o_JCamera = pin.computeFrameJacobian(model,data,q,IDX_Camera, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Camera = oMCamera.translation - oMgoalCamera.translation

    nu = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    o_Jfeet = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])

    # Tasks by order of priority
    K = 1 # convergence gain
    vq = -pinv(o_Jfeet) @ nu # feet task, -K * pinv()
    
    Pfeet = np.eye(24) - pinv(o_Jfeet) @ o_Jfeet
    vq += pinv(o_JCamera @ Pfeet) @ (-o_Camera - o_JCamera @ vq) # 2nd task

    PCamera = Pfeet - pinv(o_JCamera @ Pfeet) @ o_JCamera @ Pfeet
    vq += pinv(o_Jbase @ PCamera) @ (-o_BaseFixed - o_Jbase @ vq) # 3rd task, correct body position

    """ PBodyOrientation = PCamera - pinv(o_Jbase @ PCamera) @ o_Jbase @ PCamera
    a = o_JbaseOrientation @ PBodyOrientation
    b = -base_nu[5] - o_JbaseOrientation @ vq
    # vq += pinv(o_JbaseOrientation @ PBodyOrientation) * (-base_nu[5] - o_JbaseOrientation @ vq)
    vq += a * b """
    
    # compute the next configuration and display it
    q = pin.integrate(model, q, vq * dt)
    
    flag = False
    if norm(o_Camera) < 0.01:
        flag = True

    return q, dq, flag

def useGripper(q, dq, dt, robot, i, viz, goal=None):
    """
    position the gripper in 4D space to take something (a bottle on a table?)
    q : current configuration of the robot
    dq : current velocity of the robot
    dt : time step
    robot : Instance of the class RobotWrapper from Pinocchio
    i : indice of current main loop scutation
    viz : instance of the used vizualizer
    configuration : Pre-define configuration: choose 1, 2 or 3; To be updated with a Pinocchio::SE3 object
    goal : Pinocchio::SE3 object that indicate the target
    return : robot configuration, robot velocity, flag to know wether te action is finish or not 
    """

    global oMgoalBase, oMgoalGripper, oMgoalCamera

    model = robot.model
    data = robot.data

    # Get the robot frames
    IDX_Gripper = model.getFrameId('framegripper')
    IDX_Base = model.getFrameId('body_sasm')
    IDX_FLfoot = model.getFrameId('FL_foot_frame')
    IDX_FRfoot = model.getFrameId('FR_foot_frame')
    IDX_HLfoot = model.getFrameId('HL_foot_frame')
    IDX_HRfoot = model.getFrameId('HR_foot_frame')

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, 0.0]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, 0.0]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, 0.0]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, 0.0]))

    # define the target frame of the base and the end effector
    oMgoalBase = pin.SE3(np.eye(3), np.array([-0.1, 0.0, 0.35]))
    oMgoalGripper = goal
    if goal is None:        
        oMgoalGripper = pin.SE3(np.eye(3), np.array([0.6, 0.0, 0.3])) # pin.utils.rotate('x', np.pi/2)
        SphereGoal(viz, oMgoalGripper.translation, "GoalGripper")


    #Inverse kinematics
    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(model,data,q)
    pin.computeJointJacobians(model,data,q)

    # compute feet position error and Jacobian
    oMflfoot = data.oMf[IDX_FLfoot]
    o_Jflfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_flfoot = oMflfoot.translation - oMflfootGoal.translation

    oMfrfoot = data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfoot.translation - oMfrfootGoal.translation

    oMhlfoot = data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfoot.translation - oMhlfootGoal.translation

    oMhrfoot = data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(model, data, q, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfoot.translation - oMhrfootGoal.translation

    # base task
    oMbase = data.oMf[IDX_Base]
    o_Jbase = pin.computeFrameJacobian(model, data, q, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_BaseFixed = oMbase.translation - oMgoalBase.translation

    # Gripper task
    oMGripper = data.oMf[IDX_Gripper]
    # gripper_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    o_JGripper = pin.computeFrameJacobian(model,data,q,IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_Gripper = oMGripper.translation - oMgoalGripper.translation

    nu_feet = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    o_Jfeet = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])

    # Tasks by order of priority
    K = 10 # convergence gain
    vq = -K * pinv(o_Jfeet) @ nu_feet

    Pfeet = np.eye(robot.model.nv) - pinv(o_Jfeet) @ o_Jfeet
    vq += pinv(o_JGripper @ Pfeet) @ (-o_Gripper - o_JGripper @ vq)

    # compute the next configuration and display it
    q = pin.integrate(model, q, vq * dt)
    
    flag = False
    if norm(o_Gripper) < 0.02:
        flag = True

    return q, dq, flag