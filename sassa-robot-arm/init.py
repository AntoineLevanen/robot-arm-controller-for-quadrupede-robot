import sys
import pinocchio as pin
import numpy as np
import hppfcl
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer

def initRobot(urdfFilePath, meshFilePath):
    """
    Initialize the robot from the RobotWrapper class
    urdfFilePath : File path to URDF file
    meshFilePath : File path to mesh object files
    return : Instance of the robot containing collision model & data, visual model & data
    """
    robot = RobotWrapper.BuildFromURDF(urdfFilePath, meshFilePath, pin.JointModelFreeFlyer())
    # robot.collision_model.addAllCollisionPairs()
    
    """ height_field = hppfcl.HeightFieldOBBRSS(4, 4, np.zeros((20, 20)), 0)
    height_field_placement = pin.SE3.Identity()

    height_field = hppfcl.Box(4.0, 4.0, 0.01)
    # height_field = hppfcl.Plane(0.0, 0.0, 1.0, 0.0) # not working segmentation fault

    # print(height_field.getNodeType())

    go_height_field = pin.GeometryObject("plane", 0, height_field, height_field_placement)
    go_height_field.meshColor = np.ones(4)
    height_field_collision_id = robot.collision_model.addGeometryObject(go_height_field)
    robot.visual_model.addGeometryObject(go_height_field)

    fl_foot_collision_id = robot.collision_model.getGeometryId("gripper_0")
    go_fl_foot = robot.collision_model.geometryObjects[fl_foot_collision_id]
    go_fl_foot.geometry.buildConvexRepresentation(False)
    go_fl_foot.geometry = go_fl_foot.geometry.convex

    collision_pair = pin.CollisionPair(height_field_collision_id, fl_foot_collision_id)
    robot.collision_model.addCollisionPair(collision_pair) """

    # addCameraFrame(robot)
    addGripperFrame(robot)

    return robot # robot.collision_model, robot.collision_data, robot.visual_model, robot.visual_data

def addCameraFrame(robot):
    """
    Add frame for the camera as well as a visual in the viewport
    robot : Instance of the class RobotWrapper from Pinocchio
    """
    L = 0.05
    X = pin.utils.rotate('y', np.pi/2)
    Y = pin.utils.rotate('x', -np.pi/2)
    Z = np.eye(3)
    cyl = hppfcl.Cylinder(L/30,L)
    med = np.array([0, 0, L/2])

    FIDX = robot.model.getFrameId('OT')
    JIDX = robot.model.frames[FIDX].parent

    eff = np.array([0.2, -0.02525, 0.03])
    FIDX = robot.model.addFrame(pin.Frame('framecamera', JIDX, FIDX, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

    robot.visual_model.addGeometryObject(pin.GeometryObject('axiscamera_x', FIDX, JIDX, cyl, pin.SE3(X, X@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([1, 0, 0, 1.])

    robot.visual_model.addGeometryObject(pin.GeometryObject('axiscamera_y', FIDX, JIDX, cyl, pin.SE3(Y, Y@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([0, 1, 0, 1.])

    robot.visual_model.addGeometryObject(pin.GeometryObject('axiscamera_z', FIDX, JIDX, cyl, pin.SE3(Z, Z@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([0, 0, 1, 1.])

def addGripperFrame(robot):
    """
    Add frame for the gripper as well as a visual in the viewport
    robot : Instance of the class RobotWrapper from Pinocchio
    """
    L = 0.05
    X = pin.utils.rotate('y', np.pi/2)
    Y = pin.utils.rotate('x', np.pi)
    Z = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # np.eye(3)
    cyl = hppfcl.Cylinder(L/30,L)
    med = np.array([0, 0, L/2])

    FIDX = robot.model.getFrameId('OT')
    JIDX = robot.model.frames[FIDX].parent

    eff = np.array([0.1, -0.005, 0.03])
    FIDX = robot.model.addFrame(pin.Frame('framegripper', JIDX, FIDX, pin.SE3(Z, eff), pin.FrameType.OP_FRAME))

    robot.visual_model.addGeometryObject(pin.GeometryObject('axisgripper_x', FIDX, JIDX, cyl, pin.SE3(X, X@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([1, 0, 0, 0.5])

    robot.visual_model.addGeometryObject(pin.GeometryObject('axisgripper_y', FIDX, JIDX, cyl, pin.SE3(Y, Y@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([0, 1, 0, 0.5])

    robot.visual_model.addGeometryObject(pin.GeometryObject('axisgripper_z', FIDX, JIDX, cyl, pin.SE3(Z, Z@med+eff)))
    robot.visual_model.geometryObjects[-1].meshColor = np.array([0, 0, 1, 0.5])

    robot.data = robot.model.createData()
    robot.visual_data = robot.visual_model.createData()

def addGroundPlane(plane_URDF_path, mesh_path):
    """
    USE hppfcl to add ground plane instead, simpler to add collision detection with the robot
    """
    plane_model, plane_collision_model, plane_visual_model = pin.buildModelsFromUrdf(plane_URDF_path, mesh_path, pin.JointModelFreeFlyer())
    plane_data = plane_model.createData()
    plane_geom_model = pin.buildGeomFromUrdf(plane_model, "urdf/objects/plane.urdf", pin.GeometryType.COLLISION, package_dirs="objects")
    plane_geom_data = pin.GeometryData(plane_geom_model)
    return (plane_model, plane_collision_model, plane_visual_model, plane_data, plane_geom_model, plane_geom_data)

def addTable():
    """
    Add a visual block to serve as a table where the robot can interact with
    return : model, collision model and visual model for displaying in the viewport
    """
    box_model, box_collision_model, box_visual_model = pin.buildModelsFromUrdf("urdf/objects/box1.urdf", "urdf/objects", pin.JointModelFreeFlyer())
    return (box_model, box_collision_model, box_visual_model)

def initViz(robot, viz_choice, add_ground=False, add_box=False):
    """
    Initialize the vizualizer (MeshCat or Gepetto Viewer)
    robot : Instance of the class RobotWrapper from Pinocchio
    viz_choice : 1 -> Gepetto Viewer; 2 -> MeshCat
    add_ground : True -> add a ground in the viewport (visual only)
    add_box : True -> add a box which serve as a table
    return : instance of the visualizer
    """
    plane_model, plane_collision_model, plane_visual_model, plane_data, \
        plane_geom_model, plane_geom_data = addGroundPlane("urdf/objects/plane.urdf", "urdf/objects")

    box_model, box_collision_model, box_visual_model = addTable()

    viz = None
    if viz_choice == 1:
        viz = GepettoVisualizer(robot.model, robot.collision_model, robot.visual_model)
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
        q0 = robot.q0
        # q0[2] = 0.5
        viz.display(q0)

    elif viz_choice == 2:
        viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        try:
            viz.initViewer(open=True)
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install Python meshcat"
            )
            print(err)
            sys.exit(0)

        # Display a robot configuration.
        viz.loadViewerModel(rootNodeName="sassa")
        q0 = pin.neutral(robot.model)
        # q0[2] = 0.5
        q1 = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, -np.pi/6, \
                        np.pi/3, 0.0, -np.pi/6, np.pi/3, 0.0, 0.0, np.pi/6, -np.pi/6, 0.0])
        viz.display(q1)
        viz.displayCollisions(False) # display the collision meshes
        viz.displayVisuals(True)

        if add_ground:
            # Display the ground
            viz2 = MeshcatVisualizer(plane_model, plane_collision_model, plane_visual_model)
            viz2.initViewer(viz.viewer)
            viz2.loadViewerModel(rootNodeName="ground")
            q0 = pin.neutral(plane_model)
            # q0[2] = -0.5 # to lower the ground plane
            viz2.display(q0)

        if add_box:
            # Display a box
            viz3 = MeshcatVisualizer(box_model, box_collision_model, box_visual_model)
            viz3.initViewer(viz.viewer)
            viz3.loadViewerModel(rootNodeName="table")
            q0 = pin.neutral(box_model)
            q0[0] = 0.45
            q0[1] = -0.1
            q0[2] = 0.1
            viz3.display(q0)

    else:
        print("Chose a visualizer, 1 -> Gepetto Viewer; 2 -> MeshCat")
        sys.exit(1)
        
    return viz