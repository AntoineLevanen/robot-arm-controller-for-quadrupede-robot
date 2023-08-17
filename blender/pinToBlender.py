import bpy

## Start convenient functions
def checkConf():
  if bpy.app.version[0:2] != (2, 75):
    print("Using blender version " + str(bpy.app.version))
    print("Developed under version 2.75.0.")
    return False
  if bpy.context.scene.render.engine != 'CYCLES':
    print("Cycles renderer is prefered")
    return False
  return True

taggedObjects = list()
def tagObjects ():
  global taggedObjects
  taggedObjects = list ()
  for obj in bpy.data.objects:
    taggedObjects.append (obj.name)

def getNonTaggedObjects ():
  global taggedObjects
  return [obj for obj in bpy.data.objects if obj.name not in taggedObjects]

def setParent (children, parent):
  for child in children:
    child.parent = parent


def setMaterial (children, mat):
  for child in children:
    child.data.materials.append(mat)


def makeEmpty(objname):
  bpy.ops.object.empty_add()
  bpy.context.object.name = objname
  return bpy.context.object

def setLocQuatSca(obj, loc = None, quat=None, sca=None):
  if loc is not None: obj.location = loc
  if quat is not None: 
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = quat
  if sca is not None: obj.scale = sca

def makePolyLine(objname, curvename, cList):
  curvedata = bpy.data.curves.new(name=curvename, type='CURVE')
  curvedata.dimensions = '3D'
  curvedata.fill_mode = 'FULL'
  curvedata.bevel_depth = 0.01
  curvedata.bevel_resolution = 10

  objectdata = bpy.data.objects.new(objname, curvedata)
  objectdata.location = (0,0,0) #object origin
  bpy.context.scene.objects.link(objectdata)
  w = 1.0
  polyline = curvedata.splines.new('POLY')
  polyline.points.add(len(cList)-1)
  for num in range(len(cList)):
    x, y, z = cList[num]
    polyline.points[num].co = (x, y, z, w)
  return objectdata, curvedata


## End of convenient functions
checkConf()

bpy.ops.view3d.snap_cursor_to_center()

obj_stack = []
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//handle.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//body_panel.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//body.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_4__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_4__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_4")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//body_panel.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_5__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_5__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_5")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_6__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_6__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_6")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/body_sasm_7__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/body_sasm_7__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/body_sasm_7")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//angle_motor_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_cover.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//inline_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//motor_pulley.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_main.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_5__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_5__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_5")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//lower_leg.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//foot_ball.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//angle_motor_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_2_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_2_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_2_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_2_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_2_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_2_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//inline_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//motor_pulley.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_cover.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_main.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_4__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_4__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_4")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_5__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_2_5__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_2_5")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_2_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//foot_ball.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_2_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//lower_leg.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_2_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_2_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_2_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_3_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_3_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_3_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//angle_motor_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_3_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_3_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_3_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//inline_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_cover.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_main.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_4__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_4__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_4")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//motor_pulley.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_5__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_3_5__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_3_5")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//lower_leg.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_3_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_3_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//foot_ball.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_3_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_3_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_3_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//angle_motor_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_4_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_4_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_4_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/helbow_sasm_4_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/helbow_sasm_4_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/helbow_sasm_4_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//rmdx8v3.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_main.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//upperleg_cover.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//inline_connect.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_4__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_4__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_4")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//motor_pulley.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_5__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/upperleg_sasm_4_5__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/upperleg_sasm_4_5")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_4_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//foot_ball.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_4_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//lower_leg.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_4_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//61804-2rs.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_3__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/lowerleg_sasm_4_3__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/lowerleg_sasm_4_3")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//5015.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/arm1_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/arm1_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/arm1_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//7015.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/arm2_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/arm2_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/arm2_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//link.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/arm2_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/arm2_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/arm2_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//5015.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/arm2_sasm_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/arm2_sasm_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/arm2_sasm_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//link.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/arm3_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/arm3_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/arm3_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//end_effector.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/end_effector_sasm_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/end_effector_sasm_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/end_effector_sasm_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//t265.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/end_effector_sasm_1__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/end_effector_sasm_1__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/end_effector_sasm_1")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//2408.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/end_effector_sasm_2__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/end_effector_sasm_2__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/end_effector_sasm_2")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
tagObjects()
bpy.ops.import_scene.obj (filepath="/home/antoine/Documents/GitHub/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot-short-arm//gripper.obj", axis_forward='Y', axis_up='Z')
imported_objects = getNonTaggedObjects ()
print(imported_objects)
currentShape = makeEmpty("world/pinocchio/visuals/gripper_0__shape")
setLocQuatSca(currentShape)
setParent (imported_objects, currentShape)
currentShape.name = "world/pinocchio/visuals/gripper_0__shape"
setLocQuatSca(currentShape, loc = ( 0, 0, 0, ), quat = ( 1, 0, 0, 0, ), sca = ( 1, 1, 1, ))
obj = makeEmpty("world/pinocchio/visuals/gripper_0")
setLocQuatSca(obj)
currentShape.parent = obj
if obj_stack: obj.parent = obj_stack[-1]
