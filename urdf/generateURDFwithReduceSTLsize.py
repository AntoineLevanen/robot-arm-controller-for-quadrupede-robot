import sys, getopt
import pymeshlab as ml
import os


"""
build URDF file and reduce the size of STL file
"""

# folder path in which the file will be download
dir_path = os.path.abspath("sassa-robot")

launchBullet = False

argumentList = sys.argv[1:]
# print(len(argumentList))

# Options
options = "hbcr"

# Long options
long_options = ["Help", "Bullet", "Clear-cache", "Reduce"]

# flag to reduce or not mesh file size
reduce_mesh_file = False

try:
    # Parsing argument
    arguments, values = getopt.getopt(argumentList, options, long_options)
    
    # checking each argument
    for currentArgument, currentValue in arguments:

        if currentArgument in ("-h", "--Help"):
            print("use -b to launch PyBullet")
            print("use -c to clear caches")
            print("use -r to reduce mesh file size")
            sys.exit(2)
            
        elif currentArgument in ("-b", "--Bullet"):
            launchBullet = True

        elif currentArgument in ("-c", "--Clear-cache"):
            os.system("onshape-to-robot-clear-cache")

        elif currentArgument in ("-r", "--Reduce"):
            reduce_mesh_file = True
            
except getopt.error as err:
    # output error, and return with an error code
    print (str(err))

# download STL files and generate URDF file
os.system("onshape-to-robot " + dir_path)


# Iterate directory
for file in os.listdir(dir_path):
    # check only STL files
    if file.endswith('.stl'):
        ms = ml.MeshSet()
        ms.load_new_mesh(dir_path + "/" + file)

        if reduce_mesh_file:
            m = ms.current_mesh()
            # print('input mesh has', m.vertex_number(), 'vertex and', m.face_number(), 'faces')

            #Target number of vertex
            TARGET=2000

            #Estimate number of faces to have 100+10000 vertex using Euler
            numFaces = 100 + 2*TARGET

            #Simplify the mesh. Only first simplification will be agressive
            while (ms.current_mesh().vertex_number() > TARGET):
                ms.apply_filter('simplification_quadric_edge_collapse_decimation', targetfacenum=numFaces, preservenormal=True)
                # print("Decimated to", numFaces, "faces mesh has", ms.current_mesh().vertex_number(), "vertex")
                #Refine our estimation to slowly converge to TARGET vertex number
                numFaces = numFaces - (ms.current_mesh().vertex_number() - TARGET)

            m = ms.current_mesh()
            print('output mesh has', m.vertex_number(), 'vertex and', m.face_number(), 'faces')

        # file = file.replace('stl', 'obj') ### to be fixed
        # ms.save_current_mesh(dir_path + "/" + file)
        # print("Saving %s file..." % (file))

# load the URDF file in Bullet
if launchBullet:
    os.system("onshape-to-robot-bullet " + dir_path)

# create a temporary URDF file
robot = dir_path + "/robot.urdf"
robot2 = dir_path + "/robot_temp.urdf"

os.system("mv " + robot + " " + robot2)

# change mesh link in URDF file
print("modifying urdf file")
os.system("sed 's/\.stl/\.obj/g' %s > %s" % (robot2, robot))

# clear temporary file
print("removing temp file")
os.system("rm " + robot2)