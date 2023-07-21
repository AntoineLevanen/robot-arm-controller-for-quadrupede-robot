import matplotlib.pyplot as plt
import numpy as np
import time
from scenario1 import scenario1


def mainScenario1(info=1):
    """
    Plot the error of the CoM durring scenario 1, pick and place
    info = 1 : plot CoM error, position
    info = 2 : plot Gripper error, position
    info = 3 : plot Gripper error, velocity
    """
    log_com_1, log_goal_1, log_end_effector_1 = scenario1(robot_urdf_path="urdf/sassa-robot/robot.urdf", robot_file_path="urdf/sassa-robot/", enable_viz=False)
    log_com_2, log_goal_2, log_end_effector_2 = scenario1(robot_urdf_path="urdf/sassa-robot-short-arm/robot.urdf", robot_file_path="urdf/sassa-robot-short-arm/", enable_viz=False)

    if info == 1:
        # log test 1
        # position error of the CoM 
        fig = plt.figure()
        plt.subplot(3, 2, 1)
        e1 = [point[0] for point in log_com_1]
        plt.plot(e1, label='X CoM position')
        plt.plot(np.zeros(len(e1)), label='X CoM desired position', linestyle='dashed')
        plt.legend()
        plt.ylim([-0.03, 0.042])
        plt.title("Sassa with long arm")

        plt.subplot(3, 2, 3)
        e2 = [point[1] for point in log_com_1]
        plt.plot(e2, label='Y CoM position')
        plt.plot(np.zeros(len(e2)), label='Y CoM desired position', linestyle='dashed')
        plt.legend()
        plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 5)
        e3 = [point[2] for point in log_com_1]
        plt.plot(e3, label='Z CoM position')
        plt.legend()
        plt.ylim([0.275, 0.4])

        # log test 2
        plt.subplot(3, 2, 2)
        e1 = [point[0] for point in log_com_2]
        plt.plot(e1, label='X CoM position')
        plt.plot(np.zeros(len(e1)), label='X CoM desired position', linestyle='dashed')
        plt.legend()
        plt.ylim([-0.03, 0.042])
        plt.title("Sassa with short arm")

        plt.subplot(3, 2, 4)
        e2 = [point[1] for point in log_com_2]
        plt.plot(e2, label='Y CoM position')
        plt.plot(np.zeros(len(e2)), label='Y CoM desired position', linestyle='dashed')
        plt.legend()
        plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 6)
        e3 = [point[2] for point in log_com_2]
        plt.plot(e3, label='Z CoM position')
        plt.legend()
        plt.ylim([0.275, 0.4])

        plt.suptitle("Pick and Place operation")
        fig.supxlabel('dt = 0.01 seconds, duration = 60 seconds')
        plt.show()
    
    elif info == 2:
        # log test 1
        # Position error of the end effector
        fig = plt.figure()

        plt.subplot(3, 2, 1)
        e1 = [point[0][0]for point in log_end_effector_1]
        plt.plot(e1, label='X end effector position')
        e1 = [point[0][0] for point in log_goal_1]
        plt.plot(e1, label='X goal position', linestyle='dashed')
        plt.legend()
        plt.ylim([0.38, 0.56])
        plt.title("Sassa with long arm")

        plt.subplot(3, 2, 3)
        e2 = [point[0][1] for point in log_end_effector_1]
        plt.plot(e2, label='Y end effector position')
        e2 = [point[0][1] for point in log_goal_1]
        plt.plot(e2, label='Y goal position', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 5)
        e3 = [point[0][2] for point in log_end_effector_1]
        plt.plot(e3, label='Z end effector position')
        e3 = [point[0][2] for point in log_goal_1]
        plt.plot(e3, label='Z goal position', linestyle='dashed')
        plt.legend()
        # plt.ylim([0.275, 0.4])

        # log test 2
        plt.subplot(3, 2, 2)
        e1 = [point[0][0] for point in log_end_effector_2]
        plt.plot(e1, label='X end effector position')
        e1 = [point[0][0] for point in log_goal_2]
        plt.plot(e1, label='X goal position', linestyle='dashed')
        plt.legend()
        plt.ylim([0.38, 0.56])
        plt.title("Sassa with short arm")

        plt.subplot(3, 2, 4)
        e2 = [point[0][0] for point in log_end_effector_2]
        plt.plot(e2, label='Y end effector position')
        e2 = [point[0][1] for point in log_goal_2]
        plt.plot(e2, label='Y goal position', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 6)
        e3 = [point[0][2] for point in log_end_effector_2]
        plt.plot(e3, label='Z end effector position')
        e3 = [point[0][2] for point in log_goal_2]
        plt.plot(e3, label='Z goal position', linestyle='dashed')
        plt.legend()
        # plt.ylim([0.275, 0.4])

        plt.suptitle("Pick and Place operation position")
        fig.supxlabel('dt = 0.01 seconds, duration = 60 seconds')
        plt.show()

    elif info == 3:
        # log test 1
        # velocity error of the end effector
        fig = plt.figure()

        plt.subplot(3, 2, 1)
        e1 = [point[1][0]for point in log_end_effector_1]
        plt.plot(e1, label='X end effector velocity')
        e1 = [point[1][0] for point in log_goal_1]
        plt.plot(e1, label='X goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.03, 0.042])
        plt.title("Sassa with long arm")

        plt.subplot(3, 2, 3)
        e2 = [point[1][1] for point in log_end_effector_1]
        plt.plot(e2, label='Y end effector velocity')
        e2 = [point[1][1] for point in log_goal_1]
        plt.plot(e2, label='Y goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 5)
        e3 = [point[1][2] for point in log_end_effector_1]
        plt.plot(e3, label='Z end effector velocity')
        e3 = [point[1][2] for point in log_goal_1]
        plt.plot(e3, label='Z goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([0.275, 0.4])

        # log test 2
        plt.subplot(3, 2, 2)
        e1 = [point[1][0] for point in log_end_effector_2]
        plt.plot(e1, label='X end effector velocity')
        e1 = [point[1][0] for point in log_goal_2]
        plt.plot(e1, label='X goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.03, 0.042])
        plt.title("Sassa with short arm")

        plt.subplot(3, 2, 4)
        e2 = [point[1][0] for point in log_end_effector_2]
        plt.plot(e2, label='Y end effector velocity')
        e2 = [point[1][1] for point in log_goal_2]
        plt.plot(e2, label='Y goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([-0.01, 0.016])

        plt.subplot(3, 2, 6)
        e3 = [point[1][2] for point in log_end_effector_2]
        plt.plot(e3, label='Z end effector velocity')
        e3 = [point[1][2] for point in log_goal_2]
        plt.plot(e3, label='Z goal velocity', linestyle='dashed')
        plt.legend()
        # plt.ylim([0.275, 0.4])

        plt.suptitle("Pick and Place operation velocities")
        fig.supxlabel('dt = 0.01 seconds, duration = 60 seconds')
        plt.show()

if __name__ == "__main__":
    mainScenario1(info=2)