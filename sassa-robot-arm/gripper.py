import numpy as np
import pinocchio as pin
from numpy import deg2rad

def actuate_gripper(robot, q, dt, close=False, gripper_actuator_velocity=0.5):
    """
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    dt : time step between each iteration
    close : False gripper closed, True gripper open
    return : q, True if the action as ended, False if the ation is ongoing
    """

    if close: # open the gripper
        # print("Opening the gripper")
        # print(np.rad2deg(q[-1]))
        if q[-1] >= deg2rad(60):
            # print("Gripper open")
            q = gripper_limits(robot, q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(robot, q)
        return q, False
    
    else: # close the gripper
        # print("Clossing the gripper")
        # print(np.rad2deg(q[-1]))
        if q[-1] <= deg2rad(0):
            # print("Gripper closed")
            q = gripper_limits(robot, q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = -gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(robot, q)
        return q, False

def gripper_limits(robot, q):
    """
    Manage the gripper range of motion
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    return : The new configuration
    """
    if q[-1] < deg2rad(0):
        q[-1] = deg2rad(0)
    if q[-1] > deg2rad(60):
        q[-1] = deg2rad(60)
    
    return q



