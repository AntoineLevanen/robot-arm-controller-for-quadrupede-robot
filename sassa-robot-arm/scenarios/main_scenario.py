import matplotlib.pyplot as plt
import numpy as np
import os
from scenario1 import scenario1
from scenario2 import scenario2
from scenario3 import scenario3
from scenario4 import scenario4
from scenario5 import scenario5
from scenario6 import scenario6

"""
Choose the scenario to be played, add URDF file path and choose to vizualize the robot or not.
To use export_to_blender, enable the gepetto viewer in the scenario function (enable_to_blender=1)
Plot the mean square error of the end effector.
"""
class MainScenario:

    def __init__(self, scenario_num=1):

        urdf1 = os.path.abspath("urdf/sassa-robot/robot.urdf")
        model1 = os.path.abspath("urdf/sassa-robot/")
        urdf2 = os.path.abspath("urdf/sassa-robot-short-arm/robot.urdf")
        model2 = os.path.abspath("urdf/sassa-robot-short-arm/")
        
        # pick and place
        if scenario_num == 1:
            self.log_com_1, self.log_goal_1, self.log_end_effector_1\
                 = scenario1(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=1, export_to_blender=True)
            self.log_com_2, self.log_goal_2, self.log_end_effector_2\
                 = scenario1(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=1, export_to_blender=True)
            self.plot_title = "Pick and place"
            self.plot_path = "/home/alevanen/Documents/StageM1/ressources/scenario_1"

        # Look on top of a table
        elif scenario_num == 2:
            self.log_com_1, self.log_goal_1, self.log_end_effector_1\
                 = scenario2(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=1, export_to_blender=True)
            # self.log_com_2, self.log_goal_2, self.log_end_effector_2\
            #      = scenario2(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=1, export_to_blender=True)
            self.plot_title = "Look on top of a table"
            self.plot_path = "/home/alevanen/Documents/StageM1/ressources/scenario_2"

        # move a box around
        elif scenario_num == 3:
            self.log_com_1, self.log_goal_1, self.log_end_effector_1\
                 = scenario3(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=False, export_to_blender=False)
            self.log_com_2, self.log_goal_2, self.log_end_effector_2\
                 = scenario3(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=1, export_to_blender=False)
            self.plot_title = "Move a box around"
            self.plot_path = "/home/alevanen/Documents/StageM1/ressources/scenario_3"

        # look closely to somthing on the ground
        elif scenario_num == 4:
            # self.log_com_1, self.log_goal_1, self.log_end_effector_1\
            #      = scenario4(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=False, export_to_blender=False)
            # self.log_com_2, self.log_goal_2, self.log_end_effector_2\
            #      = scenario4(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=False, export_to_blender=False)
            # self.plot_title = "Look closely to something on the ground"
            # self.plot_path = "/home/alevanen/Documents/StageM1/ressources/scenario_4"
            pass

        # actuate lever and button
        elif scenario_num == 5:
            self.log_com_1, self.log_goal_1, self.log_end_effector_1\
                 = scenario5(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=1, export_to_blender=True)
            self.log_com_2, self.log_goal_2, self.log_end_effector_2\
                 = scenario5(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=1, export_to_blender=True)
            self.plot_title = "Actuate lever and button"
            self.plot_path = "/home/alevanen/Documents/StageM1/ressources/scenario_5"

        # actuate lever and button
        elif scenario_num == 6:
            self.log_com_1, self.log_goal_1, self.log_end_effector_1\
                 = scenario6(robot_urdf_path=urdf1, robot_file_path=model1, enable_viz=1, export_to_blender=True)
            # self.log_com_2, self.log_goal_2, self.log_end_effector_2\
            #      = scenario6(robot_urdf_path=urdf2, robot_file_path=model2, enable_viz=1, export_to_blender=True)

    def plot_figure(self, info, save_image=False):
        """
        info = 1 : plot CoM error, position
        info = 2 : plot Gripper error, position
        info = 3 : plot Gripper error, velocity
        """
        x_time_axis = np.arange(len(self.log_end_effector_1)) * 0.001

        if info == 1:
            # log test 1
            # Position error of the CoM
            fig = plt.figure()
            plt.subplot(3, 2, 1)
            e1 = [point[0][0] for point in self.log_end_effector_1]
            plt.plot(x_time_axis, e1, label='X end effector position')
            plt.plot(x_time_axis, np.ones(len(e1))*0.4, label='X end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.03, 0.042])
            plt.title("Sassa with long arm")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 3)
            e2 = [point[0][1] for point in self.log_end_effector_1]
            plt.plot(x_time_axis, e2, label='Y end effector position')
            plt.plot(x_time_axis, -np.ones(len(e1))*0.015, label='Y end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.01, 0.016])
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 5)
            e3 = [point[0][2] for point in self.log_end_effector_1]
            plt.plot(x_time_axis, e3, label='Z end effector position')
            plt.plot(x_time_axis, np.ones(len(e1))*0.35, label='Z end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.275, 0.4])
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            # log test 2
            plt.subplot(3, 2, 2)
            e1 = [point[0][0] for point in self.log_end_effector_2]
            plt.plot(x_time_axis, e1, label='X end effector position')
            plt.plot(x_time_axis, np.ones(len(e1))*0.4, label='X end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.03, 0.042])
            plt.title("Sassa with short arm")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 4)
            e2 = [point[0][1] for point in self.log_end_effector_2]
            plt.plot(x_time_axis, e2, label='Y end effector position')
            plt.plot(x_time_axis, -np.ones(len(e1))*0.015, label='Y end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.01, 0.016])
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 6)
            e3 = [point[0][2] for point in self.log_end_effector_2]
            plt.plot(x_time_axis, e3, label='Z CoM position')
            plt.plot(x_time_axis, np.ones(len(e1))*0.35, label='Z end effector desired position', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.275, 0.4])
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.suptitle(" ")

            plt.subplots_adjust(left=0.2,
                bottom=0.075,
                right=0.8,
                top=0.92,
                wspace=0.2, # 0.2
                hspace=0.37)

            plt.show()
            if save_image:
                plt.rcParams["figure.figsize"] = [7.50, 3.50]
                plt.rcParams["figure.autolayout"] = True
                plt.savefig(self.plot_path + "/CoM.png")
        
        elif info == 2:
            # log test 1
            # Position error of the end effector
            fig = plt.figure()

            plt.subplot(3, 2, 1)
            e1 = [point[0]for point in self.log_com_1]
            plt.plot(x_time_axis, e1, label='X CoM position')
            e1 = [point[0][0] for point in self.log_goal_1]
            plt.plot(x_time_axis, e1, label='X goal position', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.3, 0.56])
            plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 3)
            e2 = [point[1] for point in self.log_com_1]
            plt.plot(x_time_axis, e2, label='Y CoM position')
            e2 = [point[0][1] for point in self.log_goal_1]
            plt.plot(x_time_axis, e2, label='Y goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 5)
            e3 = [point[2] for point in self.log_com_1]
            plt.plot(x_time_axis, e3, label='Z CoM position')
            e3 = [point[0][2] for point in self.log_goal_1]
            plt.plot(x_time_axis, e3, label='Z goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Z axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")


            # log test 2
            plt.subplot(3, 2, 2)
            e1 = [point[0] for point in self.log_com_2]
            plt.plot(x_time_axis, e1, label='X CoM position')
            e1 = [point[0][0] for point in self.log_goal_2]
            plt.plot(x_time_axis, e1, label='X goal position', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.3, 0.56])
            plt.title("Sassa with short arm" + "\n" + "Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 4)
            e2 = [point[1] for point in self.log_com_2]
            plt.plot(x_time_axis, e2, label='Y CoM position')
            e2 = [point[0][1] for point in self.log_goal_2]
            plt.plot(x_time_axis, e2, label='Y goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.subplot(3, 2, 6)
            e3 = [point[2] for point in self.log_com_2]
            plt.plot(x_time_axis, e3, label='Z CoM position')
            e3 = [point[0][2] for point in self.log_goal_2]
            plt.plot(x_time_axis, e3, label='Z goal position', linestyle='dashed')
            plt.legend()
            plt.title("Position error on Z axis")
            plt.xlabel("time (s)")
            plt.ylabel("meters")

            plt.suptitle(" ")
            # fig.supxlabel("dt = 0.04 seconds")
            # plt.subplot_tool()
            plt.subplots_adjust(left=0.2,
                bottom=0.075,
                right=0.8,
                top=0.92,
                wspace=0.2, # 0.2
                hspace=0.37)
            plt.show()
            if save_image:
                plt.savefig(self.plot_path + "/EndEffector.png")

        elif info == 3:
            # log test 1
            # Velocity error of the end effector
            fig = plt.figure()

            plt.subplot(3, 2, 1)
            e1 = [point[1][0]for point in self.log_com_1]
            plt.plot(x_time_axis, e1, label='X CoM velocity')
            e1 = [point[1][0] for point in self.log_goal_1]
            plt.plot(x_time_axis, e1, label='X goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.03, 0.042])
            plt.title("Sassa with long arm")

            plt.subplot(3, 2, 3)
            e2 = [point[1][1] for point in self.log_com_1]
            plt.plot(x_time_axis, e2, label='Y CoM velocity')
            e2 = [point[1][1] for point in self.log_goal_1]
            plt.plot(x_time_axis, e2, label='Y goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.01, 0.016])

            plt.subplot(3, 2, 5)
            e3 = [point[1][2] for point in self.log_com_1]
            plt.plot(x_time_axis, e3, label='Z CoM velocity')
            e3 = [point[1][2] for point in self.log_goal_1]
            plt.plot(x_time_axis, e3, label='Z goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.275, 0.4])

            # log test 2
            plt.subplot(3, 2, 2)
            e1 = [point[1][0] for point in self.log_com_2]
            plt.plot(x_time_axis, e1, label='X CoM velocity')
            e1 = [point[1][0] for point in self.log_goal_2]
            plt.plot(x_time_axis, e1, label='X goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.03, 0.042])
            plt.title("Sassa with short arm")

            plt.subplot(3, 2, 4)
            e2 = [point[1][1] for point in self.log_com_2]
            plt.plot(x_time_axis, e2, label='Y CoM velocity')
            e2 = [point[1][1] for point in self.log_goal_2]
            plt.plot(x_time_axis, e2, label='Y goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([-0.01, 0.016])

            plt.subplot(3, 2, 6)
            e3 = [point[1][2] for point in self.log_com_2]
            plt.plot(x_time_axis, e3, label='Z CoM velocity')
            e3 = [point[1][2] for point in self.log_goal_2]
            plt.plot(x_time_axis, e3, label='Z goal velocity', linestyle='dashed')
            plt.legend()
            # plt.ylim([0.275, 0.4])

            plt.suptitle(self.plot_title)

            plt.subplots_adjust(left=0.2,
                bottom=0.075,
                right=0.8,
                top=0.92,
                wspace=0.2, # 0.2
                hspace=0.37)

            plt.show()
            if save_image:
                plt.savefig(self.plot_path + "/EndEffectorVelocity.png")

    def plot_Error(self, info, save_image=False):       
        """
        Mean square error
        info = 1 : plot CoM error, position
        info = 2 : plot Gripper error, position
        info = 3 : plot Gripper error, velocity
        """
        fig = plt.figure()

        x_time_axis = np.arange(len(self.log_end_effector_1)) * 0.001

        if info == 1:
            # plt.subplot(3, 2, 1)
            # e1 = [point[0] for point in self.log_com_1]
            # e2 = np.zeros(len(e1))
            # error_x = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_x, label='X MSE')
            # plt.legend()
            # # plt.ylim([0.3, 0.56])
            # plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")

            # plt.subplot(3, 2, 3)
            # e1 = [point[1] for point in self.log_com_1]
            # error_y = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_y, label='Y MSE')
            # plt.legend()
            # plt.title("Position error on Y axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")

            # plt.subplot(3, 2, 5)
            # e1 = [point[2] for point in self.log_com_1]
            # error_z = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_z, label='Z MSE')
            # plt.legend()
            # plt.title("Position error on Z axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")

            # # log test 2
            # plt.subplot(3, 2, 2)
            # e1 = [point[0] for point in self.log_com_2]
            # error_x = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_x, label='X MSE')
            # plt.legend()
            # plt.title("Sassa with short arm" + "\n" + "Position error on X axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")

            # plt.subplot(3, 2, 4)
            # e1 = [point[1] for point in self.log_com_2]
            # error_y = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_y, label='Y MSE')
            # plt.legend()
            # plt.title("Position error on Y axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")

            # plt.subplot(3, 2, 6)
            # e1 = [point[2] for point in self.log_com_2]
            # error_z = np.square(np.subtract(e2, e1))
            # plt.plot(x_time_axis, error_z, label='Z MSE')
            # plt.legend()

            # plt.title("Position error on Z axis")
            # plt.xlabel("time (s)")
            # plt.ylabel("Mean square error")
            # plt.suptitle(self.plot_title)
            # # plt.subplot_tool()
            # plt.subplots_adjust(left=0.125,
            #         bottom=0.075,
            #         right=0.9,
            #         top=0.92,
            #         wspace=0.45, # 0.2
            #         hspace=0.37)

            # plt.show()
            # if save_image:
            #     plt.savefig(self.plot_path + "/CoM_MSE.png")
            pass

        elif info == 2:
            plt.subplot(3, 2, 1)
            e1 = [point[0] for point in self.log_com_1]
            e2 = [point[0][0] for point in self.log_goal_1]
            error_x = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_x, label='X axis error')
            plt.legend()
            # plt.ylim([0.3, 0.56])
            plt.title("Sassa with long arm" + "\n" + "Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")

            plt.subplot(3, 2, 3)
            e1 = [point[1] for point in self.log_com_1]
            e2 = [point[0][1] for point in self.log_goal_1]
            error_y = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_y, label='Y axis error')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")

            plt.subplot(3, 2, 5)
            e1 = [point[2] for point in self.log_com_1]
            e2 = [point[0][2] for point in self.log_goal_1]
            error_z = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_z, label='Z axis error')
            plt.legend()
            plt.title("Position error on Z axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")

            # log test 2
            plt.subplot(3, 2, 2)
            e1 = [point[0] for point in self.log_com_2]
            e2 = [point[0][0] for point in self.log_goal_2]
            error_x = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_x, label='X axis error')
            plt.legend()
            plt.title("Sassa with short arm" + "\n" + "Position error on X axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")

            plt.subplot(3, 2, 4)
            e1 = [point[1] for point in self.log_com_2]
            e2 = [point[0][1] for point in self.log_goal_2]
            error_y = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_y, label='Y axis error')
            plt.legend()
            plt.title("Position error on Y axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")

            plt.subplot(3, 2, 6)
            e1 = [point[2] for point in self.log_com_2]
            e2 = [point[0][2] for point in self.log_goal_2]
            error_z = np.square(np.subtract(e2, e1))
            plt.plot(x_time_axis, error_z, label='Z axis error')
            plt.legend()

            plt.title("Position error on Z axis")
            plt.xlabel("time (s)")
            plt.ylabel("meter")
            plt.suptitle(" ")
            # plt.subplot_tool()
            plt.subplots_adjust(left=0.2,
                bottom=0.075,
                right=0.8,
                top=0.92,
                wspace=0.2, # 0.2
                hspace=0.37)

            plt.show()
            if save_image:
                plt.savefig(self.plot_path + "/EndEffector_MSE.png")

        elif info == 3:
            pass

if __name__ == "__main__":
    test1 = MainScenario(scenario_num=2)
    # for i in range(3):
    #     info_to_plot = i+1
    #     test1.plot_figure(i, save_image=False)
    #     test1.plot_Error(i, save_image=False)
