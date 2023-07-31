import pinocchio as pin
import numpy as np
import sys
import os
from pinocchio.visualize import GepettoVisualizer

"""
Pinocchio example to test Gepetto Viewer
"""
 
# Load the URDF model. 
urdf = os.path.abspath("urdf/sassa-robot/robot.urdf")
model_path = os.path.abspath("urdf/sassa-robot/")
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf, model_path, pin.JointModelFreeFlyer())
print(collision_model.geometryObjects[10].geometry)
viz = GepettoVisualizer(model=model, collision_model=collision_model, visual_model=visual_model)
 
# Initialize the viewer.
try:
    viz.initViewer(loadModel=True)
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