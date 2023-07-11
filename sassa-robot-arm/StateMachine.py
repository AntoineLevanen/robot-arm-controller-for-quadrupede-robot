import numpy as np
import pinocchio as pin
from controller import useGripper, lookAt
from gripper import actuate_gripper

class StateMahine:

    def __init__(self):
        self.current_state = 0


    def updateState(self, q, dq, dt, robot, i, viz):

        if self.current_state == 0:
            # initial state

            # take object at position 1 and open gripper
            q, dq, task_finished = useGripper(q, dq, dt, robot, i, viz, goal=pin.SE3(np.eye(3), np.array([0.45, -0.04, 0.25])))
            # update q and dq here
            q, _ = actuate_gripper(robot, q, dt, close=False)

            if task_finished:
                self.current_state = 1

        elif self.current_state == 1:
            # close gripper
            q, task_finished = actuate_gripper(robot, q, dt, close=True)
            if task_finished:
                self.current_state = 2

        elif self.current_state == 2:
            # point d'approche
            q, dq, task_finished = useGripper(q, dq, dt, robot, i, viz, goal=pin.SE3(np.eye(3), np.array([0.45, -0.1, 0.4])))

            if task_finished:
                self.current_state = 3
        
        elif self.current_state == 3:
            # insert controller here
            q, dq, task_finished = useGripper(q, dq, dt, robot, i, viz, goal=pin.SE3(np.eye(3), np.array([0.45, -0.16, 0.25])))

            if task_finished:
                self.current_state = 4

        elif self.current_state == 4:
            # open gripper
            q, task_finished = actuate_gripper(robot, q, dt, close=False)
            if task_finished:
                self.current_state = 5

        elif self.current_state == 5:
            # point d'approche
            q, dq, task_finished = useGripper(q, dq, dt, robot, i, viz, goal=pin.SE3(np.eye(3), np.array([0.5, -0.0, 0.4])))
            if task_finished:
                self.current_state = 6

        elif self.current_state == 6:
            # look behind the table
            # look at point of interest
            q, dq, task_finished = lookAt(q, dq, dt, robot, i, viz, 1)
            q, _ = actuate_gripper(robot, q, dt, close=True)

            if task_finished:
                self.current_state = 7

        elif self.current_state == 7:
            # end state, wait
            pass


        return q, dq