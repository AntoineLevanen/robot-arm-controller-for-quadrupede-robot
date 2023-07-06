import numpy as np
import pinocchio as pin
from numpy import deg2rad

def actuate_gripper(robot, q, dt, close=False):
    """
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    dt : time step between each iteration
    close : False gripper closed, True gripper open
    return : q, True if the action as ended, False if the ation is ongoing
    """
    gripper_actuator_velocity = 0.5
    q_gripper = q.copy()
    q_gripper[-1] = q_gripper[-2]
    q_gripper = gripper_limits(robot, q_gripper)

    if not close: #closing the gripper
        # print("Opening the gripper")
        if q[-1] >= deg2rad(45):
            # print("Gripper open")
            q = gripper_limits(robot, q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = gripper_actuator_velocity
        vq[-2] = -gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(robot, q)
        return q, False
    
    else: #opening the gripper
        # print("Clossing the gripper")
        if q[-1] <= deg2rad(0):
            # print("Gripper closed")
            q = gripper_limits(robot, q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = -gripper_actuator_velocity
        vq[-2] = gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(robot, q)
        return q, False


def open_gripper(robot, q):
    """
    Force the gripper to be open
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    return : The new configuration
    """
    q[-1] = deg2rad(45) # left gripper
    q[-2] = deg2rad(-45) # right gripper
    return q

def close_gripper(robot, q):
    """
    Force the gripper to be closed
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    return : The new configuration
    """
    q[-1] = deg2rad(0) # left gripper
    q[-2] = deg2rad(0) # right gripper
    return q

def gripper_limits(robot, q):
    """
    Manage the gripper range of motion
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    return : The new configuration
    """
    q_gripper = q.copy()
    # left gripper
    if q_gripper[-1] < deg2rad(0):
        q_gripper[-1] = deg2rad(0)
    if q_gripper[-1] > deg2rad(45):
        q_gripper[-1] = deg2rad(45)
    # right gripper
    if q_gripper[-2] < deg2rad(-45):
        q_gripper[-2] = deg2rad(-45)
    if q_gripper[-2] > deg2rad(0):
        q_gripper[-2] = deg2rad(0)

    return q_gripper



