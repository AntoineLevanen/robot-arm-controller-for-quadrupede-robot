import time
import math
import sys
import pinocchio as pin
import numpy as np
from numpy.linalg import inv,pinv,norm,eig,svd
from numpy import sin, cos
from computeCollision import computeCollisions
from visualObject import SphereGoal


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

def controllerCLIK2ndorder(q_current, dq_current, dt, robot, init, viz, q0_ref, goal, orientation=pin.utils.rotate('y', np.pi/2), eps=0.015, add_goal_sphere=True):
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
    
    # global sphere_goal

    # # Get the robot frames
    # IDX_Base = robot.model.getFrameId('body_sasm')
    # IDX_Gripper = robot.model.getFrameId('framegripper')
    # IDX_FLfoot = robot.model.getFrameId('FL_foot_frame')
    # IDX_FRfoot = robot.model.getFrameId('FR_foot_frame')
    # IDX_HLfoot = robot.model.getFrameId('HL_foot_frame')
    # IDX_HRfoot = robot.model.getFrameId('HR_foot_frame')

    # feet_height = 0.016

    # # define the target frame of each foot
    # oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feet_height]))
    # oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feet_height]))
    # oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feet_height]))
    # oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feet_height]))

    # # define the target frame of the end effector
    
    # goal_Gripper_position = goal[0]
    # goal_Gripper_velocity = goal[1]
    # goal_Gripper_acceleration = goal[2]

    # # target position of the end effector
    # oMgoalGripper = pin.SE3(np.eye(3), np.array([goal_Gripper_position[0], goal_Gripper_position[1], goal_Gripper_position[2]]))
    # if add_goal_sphere:
    #     if init:
    #         sphere_goal = SphereGoal(viz, oMgoalGripper.translation, "goal1")
    #     sphere_goal.moveGoalVisual(oMgoalGripper.translation)

    # # second order Closed Loop Inverse kinematics
    # # Run the algorithms that outputs values in robot.data
    # robot.forwardKinematics(q_current, v=dq_current, a=0 * dq_current)
    # pin.computeJointJacobians(robot.model,robot.data,q_current)

    # # compute feet position error and Jacobian
    # oMflfoot = robot.data.oMf[IDX_FLfoot] # get placement from world frame o to frame f
    # o_Jflfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:] # take only linear velocity
    # o_flfoot = oMflfootGoal.translation - oMflfoot.translation # desired velocity of the foot frame (position error)
    # err_vel_fl_foot = o_flfoot - (o_Jflfoot3 @ dq_current) # velocity error

    # oMfrfoot = robot.data.oMf[IDX_FRfoot]
    # o_Jfrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    # o_frfoot = oMfrfootGoal.translation - oMfrfoot.translation
    # err_vel_fr_foot = o_frfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3] # two posibility to compute the current frame velocity

    # oMhlfoot = robot.data.oMf[IDX_HLfoot]
    # o_Jhlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    # o_hlfoot = oMhlfootGoal.translation - oMhlfoot.translation
    # err_vel_hl_foot = o_hlfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    # oMhrfoot = robot.data.oMf[IDX_HRfoot]
    # o_Jhrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    # o_hrfoot = oMhrfootGoal.translation - oMhrfoot.translation
    # err_vel_hr_foot = o_hrfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    # # Gripper jacobian and error
    # oMGripper = robot.data.oMf[IDX_Gripper]
    # o_JGripper = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)
    # # get the jacobian matrix of translation part and on orientation (y axis)
    # o_JGripper = np.vstack([o_JGripper[:3], o_JGripper[4]])
    # o_Gripper = oMgoalGripper.translation - oMGripper.translation
    # err_vel_gripper = np.hstack([o_Gripper, [0, 0, 0]]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    # err_vel_gripper = np.hstack([err_vel_gripper[:3], err_vel_gripper[4]])

    # # define a rotation for the end effector
    # oMgoalGripper.rotation = orientation
    # gripper_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    # # error vector position + one orientation
    # gripper_nu = np.hstack([gripper_nu[:3], gripper_nu[4]])

    # # Base jacobian and error
    # oMBase = robot.data.oMf[IDX_Base]
    # oMgoal_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
    # base_nu = pin.log(oMBase.inverse() * oMgoal_base).vector
    # o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)
    # err_vel_base = np.array([0, 0, 0, 0, 0, 0]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector


    # # Stack the different terme in vectors to have on task for all four feet
    # e = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    # e_dot = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot])
    # J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])
    # x_ddot = np.hstack([np.zeros(12)])

    # # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot)
    # a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    # a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    # a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    # a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    # a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np
    # a_gripper = np.hstack([a_gripper[:3], a_gripper[4]])
    # a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np

    # # Stack the current acceleration of each feet frame
    # J_dot_q_dot = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])

    # # gains
    # # tune the gain to compensate the overshoot
    # K1 = 20
    # K2 = 2*np.sqrt(K1)

    # # Tasks in order of priority
    # # It is posible to scale task to affect the extremum of the velocity and acceleration, scale factor [0;1]
    # # first task with higher priority, fixe the feet on the ground 
    # d2q = pinv(J) @ (x_ddot - J_dot_q_dot + K2 * e_dot + K1 * e)

    # # Null Space of the first task
    # P0 = np.eye(robot.model.nv) - pinv(J) @ J
    # # second task with less priority, move the gripper
    # d2q += pinv(o_JGripper @ P0) @ (np.hstack([goal_Gripper_acceleration, [0]]) - a_gripper + K2 * err_vel_gripper + K1 * gripper_nu)

    # P1 = P0 - pinv(o_JGripper @ P0) @ o_JGripper @ P0
    # # constrain the CoM position in the center of the support polygone, only in X and Y
    # d2q += pinv(o_JBase[:2,:] @ P1) @ (np.array([0, 0]) - a_base[:2] + K2 * err_vel_base[:2] + K1 * base_nu[:2])


    d2q, P0 = feetTask(robot, q_current, dq_current, np.zeros(robot.model.nv), priority_order=1)

    d2q, P1, o_Gripper = gripperTask(robot, q_current, dq_current, d2q, goal, orientation, priority_order=2, null_space=P0)

    d2q, P2 = baseTask(robot, q_current, dq_current, d2q, priority_order=3, null_space=P1)


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

    flag = False
    # print(norm(o_Gripper))
    if norm(o_Gripper) < eps: # default 0.015
        # goal position is reach
        flag = True

    return q_next, dq_next, flag

def feetTask(robot, q_current, dq_current, d2q, priority_order=1, null_space=None, K1=20, K2=None):
    # get the feet frame indexes
    IDX_FLfoot = robot.model.getFrameId('FL_foot_frame')
    IDX_FRfoot = robot.model.getFrameId('FR_foot_frame')
    IDX_HLfoot = robot.model.getFrameId('HL_foot_frame')
    IDX_HRfoot = robot.model.getFrameId('HR_foot_frame')

    # define the feet heigth relative to the world
    feet_height = 0.016

    # define the target frame of each foot
    oMflfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, 0.140, feet_height]))
    oMfrfootGoal = pin.SE3(np.zeros((3,3)), np.array([0.221, -0.140, feet_height]))
    oMhlfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, 0.140, feet_height]))
    oMhrfootGoal = pin.SE3(np.zeros((3,3)), np.array([-0.221, -0.140, feet_height]))

    # compute feet position error, Jacobian and velocity error
    oMflfoot = robot.data.oMf[IDX_FLfoot] # get placement from world frame o to frame f
    o_Jflfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:] # take only linear velocity
    # there is 3 DoF so orientation is not take into account
    o_flfoot = oMflfootGoal.translation - oMflfoot.translation # desired velocity of the foot frame (position error)
    err_vel_fl_foot = o_flfoot - (o_Jflfoot3 @ dq_current) # velocity error, desired velocity to reach the goal - the current velocity

    oMfrfoot = robot.data.oMf[IDX_FRfoot]
    o_Jfrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_frfoot = oMfrfootGoal.translation - oMfrfoot.translation
    # two posibility to compute the current frame velocity :
    # - by subtracting the desired and the current velocity
    # - by using the getFrameVelocity function
    err_vel_fr_foot = o_frfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3] 

    oMhlfoot = robot.data.oMf[IDX_HLfoot]
    o_Jhlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hlfoot = oMhlfootGoal.translation - oMhlfoot.translation
    err_vel_hl_foot = o_hlfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]

    oMhrfoot = robot.data.oMf[IDX_HRfoot]
    o_Jhrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    o_hrfoot = oMhrfootGoal.translation - oMhrfoot.translation
    err_vel_hr_foot = o_hrfoot - pin.getFrameVelocity(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]
    
    # Stack the different terme in vectors to have one task for all four feet
    e = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot]) # position error
    e_dot = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot]) # velocity error
    J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3]) # feet Jacobians matrix
    x_ddot = np.hstack([np.zeros(12)]) # desired accelleration of the feet, here 0 on all axis

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) needed for the 2nd order CLIK
    a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    
    # Stack the current acceleration of each feet frame
    J_dot_q_dot = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])

    # gains
    # tune the gain to compensate the overshoot
    if K2 is None:
        K2 = 2*np.sqrt(K1)

    # compute acceleration
    P_feet = None # variable for the Null Space of this task
    if priority_order == 1:
        # compute acceleration of the joints 
        d2q = pinv(J) @ (x_ddot - J_dot_q_dot + K2 * e_dot + K1 * e)
        # compute the Null Space of this task
        P_feet = np.eye(robot.model.nv) - pinv(J) @ J
    elif null_space is not None and priority_order > 1:
        # if this task is not the first one in the stack, then the computation is slightly different
        # taking into account the Null Space of the previous tasks
        d2q += pinv(J @ null_space) @ (x_ddot - J_dot_q_dot + K2 * e_dot + K1 * e)
        P_feet = null_space @ pinv(J @ null_space) @ J @ null_space
    
    else:
        # function argument dont match, can't compute the robot acceleration
        print("Error, change priority order or add a null space matrix")
        sys.exit(0)

    return d2q, P_feet

def gripperTask(robot, q_current, dq_current, d2q, goal, orientation, priority_order=1, null_space=None, oMgoalGripper=None, K1=20, K2=None):
    # get the gripper frame indexe
    IDX_Gripper = robot.model.getFrameId('framegripper')

    # get the desired position, velocity and acceleration from the trajectory of th end effector
    goal_Gripper_position = goal[0]
    goal_Gripper_velocity = goal[1]
    goal_Gripper_acceleration = goal[2]

    # target position of the end effector
    oMgoalGripper = pin.SE3(np.eye(3), np.array(goal_Gripper_position))

    # Gripper jacobian and error
    oMGripper = robot.data.oMf[IDX_Gripper]
    o_JGripper = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)
    # get the jacobian matrix of translation part and on orientation (y axis)
    o_JGripper = np.vstack([o_JGripper[:3], o_JGripper[4]])
    o_Gripper = oMgoalGripper.translation - oMGripper.translation
    err_vel_gripper = np.hstack([o_Gripper, [0, 0, 0]]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    err_vel_gripper = np.hstack([err_vel_gripper[:3], err_vel_gripper[4]])

    # define a rotation for the end effector
    oMgoalGripper.rotation = orientation
    gripper_nu = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    # error vector position + one orientation
    gripper_nu = np.hstack([gripper_nu[:3], gripper_nu[4]])

    a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np
    a_gripper = np.hstack([a_gripper[:3], a_gripper[4]])

    # gains
    # tune the gain to compensate the overshoot
    if K2 is None:
        K2 = 2*np.sqrt(K1)
    
    # compute acceleration
    P_gripper = None # variable for the Null Space of this task
    if priority_order == 1:
        # compute acceleration of the joints 
        d2q = pinv(o_JGripper) @ (np.hstack([goal_Gripper_acceleration, [0]]) - a_gripper + K2 * err_vel_gripper + K1 * gripper_nu)
        # compute the Null Space of this task
        P_gripper = np.eye(robot.model.nv) - pinv(o_JGripper) @ o_JGripper
    elif null_space is not None:
        # if this task is not the first one in the stack, then the computation is slightly different
        # taking into account the Null Space of the previous tasks
        d2q += pinv(o_JGripper @ null_space) @ (np.hstack([goal_Gripper_acceleration, [0]]) - a_gripper + K2 * err_vel_gripper + K1 * gripper_nu)
        P_gripper = null_space @ pinv(o_JGripper @ null_space) @ o_JGripper @ null_space
    
    else:
        # function argument dont match, can't compute the robot acceleration
        print("Error, change priority order or add a null space matrix")
        sys.exit(0)
    return d2q, P_gripper, o_Gripper

def baseTask(robot, q_current, dq_current, d2q, priority_order=1, null_space=None, oMgoalGripper=None, K1=20, K2=None):
    # Get the robot frames
    IDX_Base = robot.model.getFrameId('body_sasm')

    # Base jacobian and error
    oMBase = robot.data.oMf[IDX_Base]
    oMgoal_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
    base_nu = pin.log(oMBase.inverse() * oMgoal_base).vector
    o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)
    err_vel_base = np.array([0, 0, 0, 0, 0, 0]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    # current acceleration of the base
    a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np

    # gains
    # tune the gain to compensate the overshoot
    if K2 is None:
        K2 = 2*np.sqrt(K1)
    
    # compute acceleration
    P_base = None # variable for the Null Space of this task
    if priority_order == 1:
        # compute acceleration of the joints
        d2q = pinv(o_JBase) @ (np.array([0, 0]) - a_base[:2] + K2 * err_vel_base[:2] + K1 * base_nu[:2])
        # compute the Null Space of this task
        P_base = np.eye(robot.model.nv) - pinv(o_JBase) @ o_JBase
    elif null_space is not None:
        # if this task is not the first one in the stack, then the computation is slightly different
        # taking into account the Null Space of the previous tasks
        # ajusting only the position on X and Y, Z is free. So the robot remain stable du to the fact that the CoM is in the
        # support polygone
        d2q += pinv(o_JBase[:2,:] @ null_space) @ (np.array([0, 0]) - a_base[:2] + K2 * err_vel_base[:2] + K1 * base_nu[:2])
        P_base = null_space @ pinv(o_JBase[:2,:] @ null_space) @ o_JBase[:2,:] @ null_space
    
    else:
        # function argument dont match, can't compute the robot acceleration
        print("Error, change priority order or add a null space matrix")
        sys.exit(0)

    return d2q, P_base

# not used
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
