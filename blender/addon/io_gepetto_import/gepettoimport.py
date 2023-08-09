#!/usr/bin/env python
#
# Copyright (c) 2015 CNRS
# Author: Steve Tonneau, Joseph Mirabel
#
# This file is part of hpp-gepetto-viewer.
# hpp-gepetto-viewer is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-gepetto-viewer is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-gepetto-viewer.  If not, see
# <http://www.gnu.org/licenses/>.

###
# See blender/README.md for instructions
# https://github.com/jmirabel/gepetto-viewer-corba/tree/devel/blender
###

import yaml
import os
import bpy
import bpy_extras.io_utils
from bpy.types import (Panel, Operator)
from bpy_extras.object_utils import AddObjectHelper
from bpy.props import FloatVectorProperty


def loadmotion(filename):
    with open(filename) as file:
        data = yaml.load(file, Loader=yaml.FullLoader)
        for frameId in range(len(data.keys())):
            frameKey = "frame_" + str(frameId)
            objPositions = data[frameKey]
            for objName, pos in objPositions.items():
                currentObj = bpy.context.scene.objects.get(objName)
                if currentObj:
                    currentObj.rotation_mode = "QUATERNION"
                    posF = [float(x) for x in pos]
                    currentObj.location = posF[0:3]
                    currentObj.rotation_quaternion = posF[3:7]
                    currentObj.keyframe_insert(data_path="location", frame=frameId)
                    currentObj.keyframe_insert(
                        data_path="rotation_quaternion", frame=frameId
                    )
                else:
                    print("Unknown object " + objName)


def checkframe(filename, frameId):
    with open(filename) as file:
        data = yaml.load(file)
        frameKey = "frame_" + str(frameId)
        objPositions = data[frameKey]
        for objName, pos in objPositions.items():
            currentObj = bpy.context.scene.objects.get(objName)
            if currentObj:
                currentObj.rotation_mode = "QUATERNION"
                posF = [float(x) for x in pos]
                currentObj.location = posF[0:3]
                currentObj.rotation_quaternion = posF[3:7]
            else:
                print("Unknown object " + objName)


def new_plane(mylocation, mysize, myname):
    bpy.ops.mesh.primitive_plane_add(
        size=mysize,
        calc_uvs=True,
        enter_editmode=False,
        align='WORLD',
        location=mylocation,
        rotation=(0, 0, 0),
        scale=(0, 0, 0))
    current_name = bpy.context.selected_objects[0].name
    plane = bpy.data.objects[current_name]
    plane.name = myname
    plane.data.name = myname + "_mesh"

    # adding a checker material for the ground
    MAT_NAME = "CheckerMat"
    bpy.data.materials.new(MAT_NAME)
    material = bpy.data.materials[MAT_NAME]
    material.use_nodes = True
    material.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = 0.2
    material.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (1,0,1,1)

    texture = material.node_tree.nodes.new(type='ShaderNodeTexChecker')
    texture.location[0] += -200
    texture.inputs[3].default_value = 100
    material.node_tree.links.new(texture.outputs[0], material.node_tree.nodes['Principled BSDF'].inputs['Base Color'])
    if len(plane.data.materials.items()) != 0:
        plane.data.materials.clear()
    else:
        plane.data.materials.append(material)

    return


class YamlPathImport(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
    bl_idname = "import.gepettoimport"
    bl_label = "Import a YAML Gepetto Viewer path file"

    files: bpy.props.CollectionProperty(name="File Path", type=bpy.types.OperatorFileListElement)
    directory: bpy.props.StringProperty(subtype="DIR_PATH")

    def execute(self, context):
        dir = self.directory
        for f in self.files:
            fullname = os.path.join(dir, f.name)
            self.report({"INFO"}, "Loading " + str(fullname))
            loadmotion(fullname)
        return {"FINISHED"}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class UrdfToBlendImport(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
    bl_idname = "import.urdf_to_blendimport"
    bl_label = "Import a URDF blender script"

    files: bpy.props.CollectionProperty(name="File Path", type=bpy.types.OperatorFileListElement)
    directory: bpy.props.StringProperty(subtype="DIR_PATH")

    def execute(self, context):
        dir = self.directory
        print(self.files)
        for f in self.files:
            fullname = os.path.join(dir, f.name)
            self.report({"INFO"}, "Loading " + str(fullname))
            exec(open(fullname).read())
        return {"FINISHED"}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class OBJECT_OT_add_ground_plane(Operator, AddObjectHelper):
    """Create a new Mesh Object"""
    bl_idname = "mesh.add_ground_plane"
    bl_label = "Add a ground plane"
    bl_options = {'REGISTER', 'UNDO'}

    scale: FloatVectorProperty(
        name="scale",
        default=(1.0, 1.0, 1.0),
        subtype='TRANSLATION',
        description="scaling",
    )

    def execute(self, context):

        new_plane((0, 0, 0), 40, "Ground plane")

        return {'FINISHED'}
