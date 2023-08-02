import os
import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer


class SphereGoal:

    def __init__(self, viz, coord, name):
        """
        Add a sphere to the viewport at the given location to represent the goal of the end effector
        viz : Instance of the used visualizer
        coord : Coordinate of the sphere in [X, Y, Z] w.r.t the world frame
        name : Name of the object in the viewport
        """
        self.urdfFilePath = os.path.abspath("urdf/objects/sphere.urdf")
        self.modelPath = os.path.abspath("urdf/objects")
        self.viz = viz
        
        sphere_model, sphere_collision_model, sphere_visual_model = pin.buildModelsFromUrdf(self.urdfFilePath, self.modelPath, pin.JointModelFreeFlyer())
        self.q0 = pin.neutral(sphere_model)
        
        self.viz3 = MeshcatVisualizer(sphere_model, sphere_collision_model, sphere_visual_model) 
        self.viz3.initViewer(self.viz.viewer)
        self.viz3.loadViewerModel(rootNodeName=(name))
        q = self.q0
        q[:3] = coord
        self.viz3.display(q)

    def moveGoalVisual(self, coord):
        """
        coord : New coordinate of the sphere in [X, Y, Z] w.r.t the world frame
        """
        q = self.q0
        q[:3] = coord
        self.viz3.display(q)

class CenterOfMass(SphereGoal):

    def __init__(self, viz, robot, name):
        """
        Add a small sphere to represent the projestion of com on the floor
        viz : Instance of the used visualizer
        robot : Instance of the class RobotWrapper from Pinocchio
        name : Name of the object in the viewport
        """
        super().__init__(viz, [0, 0, 0], name)
        self.model = robot.model
        self.data = robot.data

    def updatePlacement(self, q):
        """
        q : Current configuration of the robot
        """
        com_position = pin.centerOfMass(self.model, self.data, q) # compute the location of the CoM in the world
        com_projection = com_position.copy()
        com_projection[2] = 0 # nullify the Z coordinate
        super().moveGoalVisual(com_projection)
        return com_projection