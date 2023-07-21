#
# In this short script, we show how to use RobotWrapper
# integrating different kinds of viewers
#
 
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import (GepettoVisualizer, MeshcatVisualizer)
from sys import argv
import os
from os.path import dirname, join, abspath

"""
Pinocchio example to test the RobotWrapper class
"""

# If you want to visualize the robot in this example,
# you can choose which visualizer to employ
# by specifying an option from the command line:
# GepettoVisualizer: -g
# MeshcatVisualizer: -m
VISUALIZER = None
if len(argv)>1:
    opt = argv[1]
    if opt == '-g':
        VISUALIZER = GepettoVisualizer
    elif opt == '-m':
        VISUALIZER = MeshcatVisualizer
    else:
        raise ValueError("Unrecognized option: " + opt)
 
# Load the URDF model with RobotWrapper
# Conversion with str seems to be necessary when executing this file with ipython 
mesh_dir = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot/"
urdf_model_path = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot/robot.urdf"
 
robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())
 
# alias
model = robot.model
data = robot.data
 
# do whatever, e.g. compute the center of mass position expressed in the world frame
q0 = robot.q0
com = robot.com(q0)
 
# This last command is similar to:
com2 = pin.centerOfMass(model,data,q0)
 
# Show model with a visualizer of your choice
if VISUALIZER:
    robot.setVisualizer(VISUALIZER())
    robot.initViewer()
    robot.loadViewerModel("pinocchio")
    robot.display(q0)