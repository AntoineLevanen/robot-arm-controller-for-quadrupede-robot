bl_info = {
    "author": "Joseph Mirabel",
    "name": "Gepetto Viewer Blender Addon",
    "category": "Import-Export",
    "blender": (3, 6, 1),
    "description": "Add functionality to import files generated "
    "using the Gepetto Viewer software",
    "location": "View3D > N",
    "wiki_url": "https://github.com/jmirabel/gepetto-viewer-corba/tree/master/blender",
    "warning": "Not heavily tested, feel free to report bug on github.",
    "version": (0, 0),
    "support": "COMMUNITY",
}

import bpy
from .gepettoimport_copy import (UrdfToBlendImport, YamlPathImport)


class CustomPanel(bpy.types.Panel):
    bl_label = "Gepetto Viewer Panel"
    bl_idname = "OBJECT_PT_gepetto"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Gepetto Viewer"
    
    def draw(self, context):
        layout = self.layout
        obj = context.object
        row = layout.row()
        column = layout.column()
        row.operator(UrdfToBlendImport.bl_idname, text="Import URDF file", icon='FILE_SCRIPT')
        column.operator(YamlPathImport.bl_idname, text="Import YAML file", icon='FILE')
   

def register():
    bpy.utils.register_class(CustomPanel)
    bpy.utils.register_class(YamlPathImport)
    bpy.utils.register_class(UrdfToBlendImport)

def unregister():
    bpy.utils.unregister_class(CustomPanel)
    bpy.utils.unregister_class(YamlPathImport)
    bpy.utils.unregister_class(UrdfToBlendImport)