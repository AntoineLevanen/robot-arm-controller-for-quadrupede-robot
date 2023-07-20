import pinocchio as pin
import numpy as np
import sys
import os
from pinocchio.visualize import GepettoVisualizer
 
# Load the URDF model. 
urdf = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot/robot.urdf" # "urdf/sassa-robot/robot.urdf"
model_path = "/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot/" # "urdf/sassa-robot/"
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf, model_path, pin.JointModelFreeFlyer())
viz = GepettoVisualizer(model, collision_model, visual_model)
 
# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install gepetto-viewer")
    print(err)
    sys.exit(0)
 
try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print("Error while loading the viewer model. It seems you should start gepetto-viewer")
    print(err)
    sys.exit(0)
 
# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

while True:
    # Display a robot configuration.
    q0 = pin.neutral(model)
    viz.display(q0)