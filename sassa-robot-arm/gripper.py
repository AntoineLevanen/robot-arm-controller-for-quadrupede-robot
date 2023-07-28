import numpy as np
import pinocchio as pin
from numpy import deg2rad

def actuate_gripper(robot, q, dt, gripper_actuator_velocity=0.5, action=None):
    """
    robot : Instance of the class RobotWrapper from Pinocchio
    q : Current configuration of the robot
    dt : time step between each iteration
    action : "open"; "close"; a value between 0 and 60 degree
    return : q, True if the action as ended, False if the ation is ongoing
    """

    if action == "open": # open the gripper
        if q[-1] >= deg2rad(59.6):
            q = gripper_limits(q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(q)
        return q, False
    
    elif action == "close": # close the gripper
        if q[-1] <= deg2rad(0):
            q = gripper_limits(q)
            return q, True
        vq = np.zeros(robot.model.nv)
        vq[-1] = -gripper_actuator_velocity
        q = pin.integrate(robot.model, q, vq * dt)
        q = gripper_limits(q)
        return q, False

    elif type(action) is int or type(action) is float:
        if action > 0:
            if q[-1] == deg2rad(action):
                q = gripper_limits(q)
                return q, True
            vq = np.zeros(robot.model.nv)
            vq[-1] = gripper_actuator_velocity
            q = pin.integrate(robot.model, q, vq * dt)
            q = gripper_limits(q)
            return q, False
        else:
            if q[-1] == deg2rad(action):
                q = gripper_limits(q)
                return q, True
            vq = np.zeros(robot.model.nv)
            vq[-1] = -gripper_actuator_velocity
            q = pin.integrate(robot.model, q, vq * dt)
            q = gripper_limits(q)
            return q, False

    else:
        print("action parameter must be wether : \n - \"open\" \n - \"close\" \n - value between 0 and 60 ")
        return q, False


def gripper_limits(q):
    """
    Manage the gripper range of motion
    q : Current configuration of the robot
    return : The new configuration
    """
    if q[-1] < deg2rad(0):
        q[-1] = deg2rad(0)
    if q[-1] > deg2rad(60):
        q[-1] = deg2rad(60)
    
    return q



