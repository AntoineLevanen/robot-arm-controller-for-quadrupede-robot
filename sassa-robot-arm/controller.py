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
    x_flfoot = robot.data.oMf[IDX_FLfoot]
    J_flfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_FLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    e_flfoot = x_flfoot.translation - oMflfootGoal.translation

    x_frfoot = robot.data.oMf[IDX_FRfoot]
    J_frfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_FRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    e_frfoot = x_frfoot.translation - oMfrfootGoal.translation

    x_hlfoot = robot.data.oMf[IDX_HLfoot]
    J_hlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_HLfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    e_hlfoot = x_hlfoot.translation - oMhlfootGoal.translation

    x_hrfoot = robot.data.oMf[IDX_HRfoot]
    J_hrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_HRfoot, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    e_hrfoot = x_hrfoot.translation - oMhrfootGoal.translation

    # Gripper task
    oMGripper = robot.data.oMf[IDX_Gripper]
    # e_gripper = pin.log(oMGripper.inverse() * oMgoalGripper).vector
    J_gripper = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    x_gripper = oMGripper.translation - oMgoalGripper.translation

    e_feet = np.hstack([e_flfoot, e_frfoot, e_hlfoot, e_hrfoot])
    J_feet = np.vstack([J_flfoot3, J_frfoot3, J_hlfoot3, J_hrfoot3])

    # Tasks in order of priority
    K1 = 1
    vq = K1 * pinv(J_feet) @ e_feet # first task, fixe the feet
    
    null_space_feet = np.eye(24) - pinv(J_feet) @ J_feet
    vq += pinv(J_gripper @ null_space_feet) @ (-x_gripper - J_gripper @ vq) # second task, move the gripper

    null_space_gripper = null_space_feet - pinv(J_gripper @ null_space_feet) @ J_gripper @ null_space_feet

    # compute the next configuration and display it
    q = pin.integrate(robot.model, q, vq * dt)

    return q, vq

def controllerCLIK2ndorder(q, q_dot, dt, robot, init, viz, q0_ref, goal, orientation=pin.utils.rotate('y', 0), eps=0.015, add_goal_sphere=True):
    """
    The gripper follow the desired trajectory using second order closed loop inverse kinematics (CLIK)
    Controlling Position and Orientation of the end effector with the four leg stick to the ground, 
    the body will adapte to follow the end effector movement
    q_current : current configuration of the robot
    q_dot_current : current velocity of the robot
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

    pin.forwardKinematics(robot.model, robot.data, q, q_dot, q_dot * 0)
    pin.computeJointJacobians(robot.model, robot.data, q) # Also compute operational frame placements

    ### feet part
    # Get the robot frames
    index_end_effector = robot.model.getFrameId('framegripper')
    index_fl_foot = robot.model.getFrameId('FL_foot_frame')
    index_fr_foot = robot.model.getFrameId('FR_foot_frame')
    index_hl_foot = robot.model.getFrameId('HL_foot_frame')
    index_hr_foot = robot.model.getFrameId('HR_foot_frame')

    # define the target frame of each foot
    feet_height = 0.016
    x_star_fl_foot = pin.SE3(np.eye(3), np.array([0.221, 0.140, feet_height]))
    x_star_fr_foot = pin.SE3(np.eye(3), np.array([0.221, -0.140, feet_height]))
    x_star_hl_foot = pin.SE3(np.eye(3), np.array([-0.221, 0.140, feet_height]))
    x_star_hr_foot = pin.SE3(np.eye(3), np.array([-0.221, -0.140, feet_height]))

    # compute feet position error and Jacobian
    x_fl_foot = robot.data.oMf[index_fl_foot] # get placement from world frame o to frame f
    # take only linear velocity
    J_fl_foot3 = pin.computeFrameJacobian(robot.model, robot.data, q, index_fl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    error_fl_foot = x_star_fl_foot.translation - x_fl_foot.translation # position error
    error_dot_fl_foot = np.zeros(3) - (J_fl_foot3 @ q_dot) # velocity error

    x_fr_foot = robot.data.oMf[index_fr_foot]
    J_fr_foot3 = pin.computeFrameJacobian(robot.model, robot.data, q, index_fr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    error_fr_foot = x_star_fr_foot.translation - x_fr_foot.translation
    error_dot_fr_foot = np.zeros(3) - (J_fr_foot3 @ q_dot)

    x_hl_foot = robot.data.oMf[index_hl_foot]
    J_hl_foot3 = pin.computeFrameJacobian(robot.model, robot.data, q, index_hl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    error_hl_foot = x_star_hl_foot.translation - x_hl_foot.translation
    error_dot_hl_foot = np.zeros(3) - (J_hl_foot3 @ q_dot)

    x_hr_foot = robot.data.oMf[index_hr_foot]
    J_hr_foot3 = pin.computeFrameJacobian(robot.model, robot.data, q, index_hr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    error_hr_foot = x_star_hr_foot.translation - x_hr_foot.translation
    error_dot_hr_foot = np.zeros(3) - (J_hr_foot3 @ q_dot)

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) in the 2ndCLIK equation
    Jdot_qdot_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_fl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_fr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_hl_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_hr_foot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

    # Stack the different terme in vectors to have one task for all four feet
    error_feet = np.hstack([error_fl_foot, error_fr_foot, error_hl_foot, error_hr_foot])
    error_dot_feet = np.hstack([error_dot_fl_foot, error_dot_fr_foot, error_dot_hl_foot, error_dot_hr_foot])
    J_feet = np.vstack([J_fl_foot3, J_fr_foot3, J_hl_foot3, J_hr_foot3])
    Jdot_qdot_feet = np.hstack([Jdot_qdot_fl_foot, Jdot_qdot_fr_foot, Jdot_qdot_hl_foot, Jdot_qdot_hr_foot])
    # x_ddot_feet is indeed the term Jdot_qdot computed by Pinocchio
    x_star_ddot_feet = np.zeros(12)

    ### base part
    index_base = robot.model.getFrameId('body_sasm')
    J_base = pin.computeFrameJacobian(robot.model, robot.data, q, index_base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    Jdot_qdot_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    x_base = robot.data.oMf[index_base]
    x_star_base = pin.SE3(np.eye(3), np.array([0, 0, 0.35]))
    x_dot_base = J_base @ q_dot
    x_star_dot_base = np.zeros(3)
    # x_ddot_base is indeed the term Jdot_qdot computed by Pinocchio
    x_star_ddot_base = np.zeros(3)

    error_base = x_star_base.translation - x_base.translation
    error_dot_base = x_star_dot_base - x_dot_base

    ### end effector part
    trajectory_point = goal
    trajectory_point_position = trajectory_point[0]
    trajectory_point_velocity = trajectory_point[1]
    trajectory_point_acceleration = trajectory_point[2]

    # orientation = pin.utils.rotate('y', 0)

    J_end_effector = pin.computeFrameJacobian(robot.model, robot.data, q, index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_end_effector = np.vstack([J_end_effector[:3,:], J_end_effector[4,:]])
    Jdot_qdot_end_effector = pin.getFrameClassicalAcceleration(robot.model, robot.data, index_end_effector, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    Jdot_qdot_end_effector = np.hstack([Jdot_qdot_end_effector[:3], Jdot_qdot_end_effector[4]])
    
    x_end_effector = robot.data.oMf[index_end_effector] # Get placement from world frame o to frame f oMf
    x_star_end_effector = pin.SE3(orientation, np.array(trajectory_point_position))
    x_dot_end_effector = J_end_effector @ q_dot
    x_star_dot_end_effector = np.hstack([trajectory_point_velocity, 0])
    # x_ddot_end_effector is indeed the term Jdot_qdot computed by Pinocchio
    x_star_ddot_end_effector = np.hstack([trajectory_point_acceleration, 0])

    # error_end_effector = x_star_end_effector.translation - x_end_effector.translation
    error_end_effector = pin.log(x_end_effector.inverse() * x_star_end_effector).vector
    error_end_effector = np.hstack([error_end_effector[:3], error_end_effector[4]])
    error_dot_end_effector = x_star_dot_end_effector - x_dot_end_effector
    
    ### computing the acceleration vector
    kp = 10
    kd = 2*np.sqrt(kp)

    # combining end effector task with the feet one
    J = np.vstack([J_end_effector, J_feet])
    x_star_ddot = np.hstack([x_star_ddot_end_effector, x_star_ddot_feet])
    Jdot_qdot = np.hstack([Jdot_qdot_end_effector, Jdot_qdot_feet])
    error_dot = np.hstack([error_dot_end_effector, error_dot_feet])
    error = np.hstack([error_end_effector, error_feet])

    # end effector and feet task separated
    # q_ddot = pinv(J_end_effector) @ (x_star_ddot_end_effector - Jdot_qdot_end_effector + kd * error_dot_end_effector + kp * error_end_effector)
    # P0 = np.eye(robot.model.nv) - pinv(J_end_effector) @ J_end_effector
    # q_ddot += pinv(J_feet @ P0) @ (x_star_ddot_feet - Jdot_qdot_feet + kd * error_dot_feet + kp * error_feet)
    # P1 = P0 - pinv(J_feet @ P0) @ J_feet @ P0
    q_ddot = pinv(J) @ (x_star_ddot - Jdot_qdot + kd * error_dot + kp * error)
    P1 = np.eye(robot.model.nv) - pinv(J) @ J
    kp = 1
    kd = 2*np.sqrt(kp)
    q_ddot += pinv(J_base[:3] @ P1) @ (x_star_ddot_base[:3] - Jdot_qdot_base[:3] + kd * error_dot_base[:3] + kp * error_base[:3])

    # integrate the acceleration to get the velocity
    q_dot += q_ddot * dt
    
    # integrate the velocity to get the angular position of each joint
    q = pin.integrate(robot.model, q, q_dot * dt)


    flag = False
    # # print(norm(e_gripper))
    # if norm(e_gripper) < eps: # default 0.015
    #     # the goal position is reach
    #     flag = True

    return q, q_dot, flag

def controllerCLIK2ndorderBase(q_current, q_dot_current, dt, robot, init, viz, q0_ref, goal, orientation=pin.utils.rotate('y', np.pi/2), eps=0.015, add_goal_sphere=True, base_task=None):
    """
    The gripper follow the desired trajectory using second order closed loop inverse kinematics (CLIK)
    Controlling Position and Orientation of the end effector with the four leg stick to the ground, 
    the body will adapte to follow the end effector movement
    q_current : current configuration of the robot
    q_dot_current : current velocity of the robot
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
    x_star_gripper = pin.SE3(np.eye(3), np.array(gripper))
    if add_goal_sphere:
        if init:
            sphere_goal = SphereGoal(viz, x_star_gripper.translation, "goal1")
        sphere_goal.moveGoalVisual(x_star_gripper.translation)

    # second order Closed Loop Inverse Kinematics, 2nd order CLIK

    # Run the algorithms that outputs values in robot.data
    robot.forwardKinematics(q=q_current, v=q_dot_current, a=q_dot_current * 0)
    pin.framesForwardKinematics(robot.model, robot.data, q=q_current)
    # compute feet position error and Jacobian
    x_flfoot = robot.data.oMf[IDX_FLfoot] # get placement from world frame o to frame f
    J_flfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:] # take only linear velocity
    e_flfoot = oMflfootGoal.translation - x_flfoot.translation # position error
    e_dot_fl_foot = e_flfoot - (J_flfoot3 @ q_dot_current) # velocity error

    x_frfoot = robot.data.oMf[IDX_FRfoot]
    J_frfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    e_frfoot = oMfrfootGoal.translation - x_frfoot.translation
    e_dot_fr_foot = e_frfoot - (J_frfoot3 @ q_dot_current)

    x_hlfoot = robot.data.oMf[IDX_HLfoot]
    J_hlfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    e_hlfoot = oMhlfootGoal.translation - x_hlfoot.translation
    e_dot_hl_foot = e_hlfoot - (J_hlfoot3 @ q_dot_current)

    x_hrfoot = robot.data.oMf[IDX_HRfoot]
    J_hrfoot3 = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
    e_hrfoot = oMhrfootGoal.translation - x_hrfoot.translation
    e_dot_hr_foot = e_hrfoot - (J_hrfoot3 @ q_dot_current)


    # Gripper jacobian and error
    J_gripper = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Gripper, pin.LOCAL_WORLD_ALIGNED)
    # get the jacobian matrix of translation part and on orientation (y axis)
    J_gripper = np.vstack([J_gripper[:3], J_gripper[4]]) # jacobian matrix for xyz and one orientation
    # e_gripper = x_star_gripper.translation - x_gripper.translation # target pos - current pos
    # define a rotation for the end effector
    x_gripper = robot.data.oMf[IDX_Gripper]
    x_star_gripper.rotation = orientation
    x_dot_gripper = J_gripper @ q_dot_current
    x_star_dot_fripper = np.hstack([gripper_dot, 0])

    e_gripper = pin.log(x_gripper.inverse() * x_star_gripper).vector
    e_gripper = np.hstack([e_gripper[:3], e_gripper[4]])
    e_dot_gripper = x_star_dot_fripper - x_dot_gripper
    e_dot_gripper = np.hstack([e_dot_gripper, 0])


    # Base jacobian and error
    x_base = robot.data.oMf[IDX_Base]
    x_star__base = pin.SE3(np.eye(3), np.array(base_task[0]))
    e_base = x_star__base.translation - x_base.translation
    J_base = pin.computeFrameJacobian(robot.model, robot.data, q_current, IDX_Base, pin.LOCAL_WORLD_ALIGNED)[:3,:]
    x_star_dot_base = base_task[1]
    e_dot_base = x_star_dot_base - (J_base @ q_dot_current)


    # Stack the different terme in vectors to have on task for all four feet
    e_feet = np.hstack([e_flfoot, e_frfoot, e_hlfoot, e_hrfoot])
    e_dot_feet = np.hstack([e_dot_fl_foot, e_dot_fr_foot, e_dot_hl_foot, e_dot_hr_foot])
    J_feet = np.vstack([J_flfoot3, J_frfoot3, J_hlfoot3, J_hrfoot3])

    # getFrameClassicalAcceleration give directly the terme (J_dot(q, q_dot) * q_dot) in the 2ndCLIK equation
    Jdot_qdot_fl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_fr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_FRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_hl_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HLfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_hr_foot = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_HRfoot, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
    Jdot_qdot_gripper = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Gripper, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
    Jdot_qdot_gripper = np.hstack([Jdot_qdot_gripper[:3], Jdot_qdot_gripper[4]])
    Jdot_qdot_base = pin.getFrameClassicalAcceleration(robot.model, robot.data, IDX_Base, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

    # Stack the current acceleration of each feet frame
    Jdot_qdot_feet = np.hstack([Jdot_qdot_fl_foot, Jdot_qdot_fr_foot, Jdot_qdot_hl_foot, Jdot_qdot_hr_foot])

    # gains
    K1 = 1
    K2 = 2*np.sqrt(K1)

    # Tasks in order of priority
    # It is posible to scale task to affect the max and min of the velocity and acceleration, scale factor [0;1]

    # first task with higher priority, fixe the feet on the ground 
    x_star_ddot_feet = np.hstack([np.zeros(12)])
    d2q = pinv(J_feet) @ (x_star_ddot_feet - Jdot_qdot_feet + K2 * e_dot_feet + K1 * e_feet)

    # Null Space of the first task
    P0 = np.eye(robot.model.nv) - pinv(J_feet) @ J_feet
    # second task with less priority, move the gripper
    # d2q += pinv(J_gripper[:3] @ P0) @ (np.hstack([gripper_ddot - Jdot_qdot_gripper[:3], 0])[:3] + K2 * e_dot_gripper[:3] + K1 * e_gripper[:3])

    
    P1 = P0 - pinv(J_gripper[:3] @ P0) @ J_gripper[:3] @ P0
    # constrain the CoM position in the center of the support polygone, only in X and Y
    x_star_ddot_base = base_task[2]
    d2q += pinv(J_base @ P1) @ (x_star_ddot_base - Jdot_qdot_base + K2 * e_dot_base + K1 * e_base)


    # # Add a Regulation Task to fill the free remaining dof
    # # computing the error in position in the configuration space base : xyz,abc
    # q_temp = q0_ref - q_current
    # q_temp = np.hstack([[0, 0, 0, 0, 0, 0], q_temp[7:]])

    # K3 = 1
    # J_posture = np.eye(robot.model.nv)
    # J_posture[:6, :6] = 0
    # # d2q += K3 * J_posture @ q_temp


    # compute the joints velocity
    dq_next = d2q * dt + q_dot_current

    # compute the next configuration
    q_next = pin.integrate(robot.model, q_current, dq_next * dt)


    flag = False
    # print(norm(e_gripper))
    if norm(e_gripper) < eps: # default 0.015
        # the goal position is reach
        flag = True

    return q_next, dq_next, flag