import time
import sys
import pinocchio as pin
import numpy as np
from numpy.linalg import inv,pinv,norm
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

def controllerCLIK2ndorder(q_current, dq_current, dt, robot, init, viz, q0_ref, goal, orientation=pin.utils.rotate('y', np.pi/2), eps=0.015, add_goal_sphere=True, base_task=None):
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
    orientation : orientation of the end effector along end effector Y axis
    eps : error threashold
    add_goal_sphere : adding a goal sphere that that follow the exact goal trajectory
    return : robot configuration, robot velocity, if the task is finish or not
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
    gripper = goal[0]
    gripper_dot = goal[1]
    gripper_ddot = goal[2]

    # target position of the end effector
    oMgoalGripper = pin.SE3(np.eye(3), np.array(gripper))
    if add_goal_sphere:
        if init:
            sphere_goal = SphereGoal(viz, oMgoalGripper.translation, "goal1")
        sphere_goal.moveGoalVisual(oMgoalGripper.translation)

    # second order Closed Loop Inverse Kinematics, 2nd order CLIK

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
    o_JGripper = np.vstack([o_JGripper[:3], o_JGripper[4]]) # jacobian matrix for xyz and one orientation
    e_gripper = oMgoalGripper.translation - oMGripper.translation # traget pos - current pos
    e_dot_gripper = np.hstack([e_gripper, [0, 0, 0]]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    # e_dot_gripper = e_gripper - (o_JGripper[:3,:] @ dq_current)
    e_dot_gripper = np.hstack([e_dot_gripper[:3], e_dot_gripper[4]])

    # define a rotation for the end effector
    oMgoalGripper.rotation = orientation
    e_gripper = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    # error vector position + one orientation
    e_gripper = np.hstack([e_gripper[:3], e_gripper[4]])


    # Base jacobian and error
    oMBase = robot.data.oMf[IDX_Base]
    oMgoal_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
    if base_task is not None:
        oMgoal_base.translation = base_task[0]
    base_nu = pin.log(oMBase.inverse() * oMgoal_base).vector
    o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)
    err_vel_base = np.array([0, 0, 0, 0, 0, 0]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    if base_task is not None:
        o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        err_vel_base = base_task[1] - pin.getFrameVelocity(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector[:3]


    # Stack the different terme in vectors to have on task for all four feet
    e_feet = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    e_dot_feet = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot])
    J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])
    feet_ddot = np.hstack([np.zeros(12)])

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) in the 2ndCLIK equation
    a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    a_gripper = np.hstack([a_gripper[:3], a_gripper[4]])
    a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).np

    # Stack the current acceleration of each feet frame
    a_feet = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])



    # gains
    K1 = 1
    K2 = 1 # 2*np.sqrt(K1)

    # Tasks in order of priority
    # It is posible to scale task to affect the max and min of the velocity and acceleration, scale factor [0;1]

    # first task with higher priority, fixe the feet on the ground 
    d2q = pinv(J) @ (feet_ddot - a_feet + K2 * e_dot_feet + K1 * e_feet)
    # print(e_feet[:3])

    # Null Space of the first task
    P0 = np.eye(robot.model.nv) - pinv(o_JGripper) @ o_JGripper
    # second task with less priority, move the gripper
    d2q += pinv(o_JGripper @ P0) @ (np.hstack([gripper_ddot, [0]]) - a_gripper + K2 * e_dot_gripper + K1 * e_gripper)

    # d2q = pinv(o_JGripper[:3,:]) @ (gripper_ddot - a_gripper + K2 * e_dot_gripper[:3] + K1 * e_gripper)

    
    P1 = P0 - pinv(o_JGripper @ P0) @ o_JGripper @ P0
    # constrain the CoM position in the center of the support polygone, only in X and Y
    if base_task is not None:
        d2q += pinv(o_JBase @ P1) @ (base_task[2] - a_base[:3] + K2 * err_vel_base + K1 * base_nu[:3])
    else:
        d2q += pinv(o_JBase[:2,:] @ P1) @ (np.array([0, 0]) - a_base[:2] + K2 * err_vel_base[:2] + K1 * base_nu[:2])



    # Add a Regulation Task to fill the free remaining dof
    # computing the error in position in the configuration space base : xyz,abc
    q_temp = q0_ref - q_current
    q_temp = np.hstack([[0, 0, 0, 0, 0, 0], q_temp[7:]])

    K3 = 1
    J_posture = np.eye(robot.model.nv)
    J_posture[:6, :6] = 0
    # d2q += K3 * J_posture @ q_temp



    # compute the joints velocity
    dq_next = d2q * dt

    # compute the next configuration
    q_next = pin.integrate(robot.model, q_current, dq_next * dt)



    flag = False
    # print(norm(e_gripper))
    if norm(e_gripper) < eps: # default 0.015
        # the goal position is reach
        flag = True

    return q_next, dq_next, flag


def controllerCLIK2ndorderBase(q_current, dq_current, dt, robot, init, viz, q0_ref, goal, orientation=pin.utils.rotate('y', np.pi/2), eps=0.015, add_goal_sphere=True, base_task=None):
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
    orientation : orientation of the end effector along end effector Y axis
    eps : error threashold
    add_goal_sphere : adding a goal sphere that that follow the exact goal trajectory
    return : robot configuration, robot velocity, if the task is finish or not
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
    gripper = goal[0]
    gripper_dot = goal[1]
    gripper_ddot = goal[2]

    # target position of the end effector
    oMgoalGripper = pin.SE3(np.eye(3), np.array(gripper))
    if add_goal_sphere:
        if init:
            sphere_goal = SphereGoal(viz, oMgoalGripper.translation, "goal1")
        sphere_goal.moveGoalVisual(oMgoalGripper.translation)

    # second order Closed Loop Inverse Kinematics, 2nd order CLIK

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
    o_JGripper = np.vstack([o_JGripper[:3], o_JGripper[4]]) # jacobian matrix for xyz and one orientation
    e_gripper = oMgoalGripper.translation - oMGripper.translation # traget pos - current pos
    e_dot_gripper = np.hstack([e_gripper, [0, 0, 0]]) - pin.getFrameVelocity(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    # e_dot_gripper = e_gripper - (o_JGripper[:3,:] @ dq_current)
    e_dot_gripper = np.hstack([e_dot_gripper[:3], e_dot_gripper[4]])

    # define a rotation for the end effector
    oMgoalGripper.rotation = orientation
    e_gripper = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    # error vector position + one orientation
    e_gripper = np.hstack([e_gripper[:3], e_gripper[4]])


    # Base jacobian and error
    oMBase = robot.data.oMf[IDX_Base]
    oMgoal_base = pin.SE3(np.eye(3), base_task[0])
    e_base = oMgoal_base.translation - oMBase.translation
    o_JBase = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:3, :]
    e_dot_base = base_task[1] - (o_JBase @ dq_current)


    # Stack the different terme in vectors to have on task for all four feet
    e_feet = np.hstack([o_flfoot, o_frfoot, o_hlfoot, o_hrfoot])
    e_dot_feet = np.hstack([err_vel_fl_foot, err_vel_fr_foot, err_vel_hl_foot, err_vel_hr_foot])
    J = np.vstack([o_Jflfoot3, o_Jfrfoot3, o_Jhlfoot3, o_Jhrfoot3])
    feet_ddot = np.hstack([np.zeros(12)])

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) in the 2ndCLIK equation
    a_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    a_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    a_gripper = np.hstack([a_gripper[:3], a_gripper[4]])
    a_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

    # Stack the current acceleration of each feet frame
    a_feet = np.hstack([a_fl_foot, a_fr_foot, a_hl_foot, a_hr_foot])



    # gains
    K1 = 60
    K2 = 2*np.sqrt(K1)

    # Tasks in order of priority
    # It is posible to scale task to affect the max and min of the velocity and acceleration, scale factor [0;1]

    # first task with higher priority, fixe the feet on the ground 
    d2q = pinv(J) @ (feet_ddot - a_feet + K2 * e_dot_feet + K1 * e_feet)
    print(e_feet[:3])

    # Null Space of the first task
    P0 = np.eye(robot.model.nv) - pinv(J) @ J
    # second task with less priority, move the gripper
    # d2q += pinv(o_JGripper @ P0) @ (np.hstack([gripper_ddot, [0]]) - a_gripper + K2 * e_dot_gripper + K1 * e_gripper)

    d2q = pinv(o_JGripper @ P0) @ (np.hstack([gripper_ddot - a_gripper[:3], 0]) + K2 * e_dot_gripper + K1 * e_gripper)

    
    P1 = P0 - pinv(o_JGripper[:3,:] @ P0) @ o_JGripper[:3,:] @ P0
    # constrain the CoM position in the center of the support polygone, only in X and Y
    d2q = pinv(o_JBase @ P1) @ (base_task[2] - a_base + K2 * e_dot_base + K1 * e_base)


    # Add a Regulation Task to fill the free remaining dof
    # computing the error in position in the configuration space base : xyz,abc
    q_temp = q0_ref - q_current
    q_temp = np.hstack([[0, 0, 0, 0, 0, 0], q_temp[7:]])

    K3 = 1
    J_posture = np.eye(robot.model.nv)
    J_posture[:6, :6] = 0
    # d2q += K3 * J_posture @ q_temp



    # compute the joints velocity
    dq_next = d2q * dt

    # compute the next configuration
    q_next = pin.integrate(robot.model, q_current, dq_next * dt)



    flag = False
    # print(norm(e_gripper))
    if norm(e_gripper) < eps: # default 0.015
        # the goal position is reach
        flag = True

    return q_next, dq_next, flag