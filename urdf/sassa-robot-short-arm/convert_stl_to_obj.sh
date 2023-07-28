#Convert all mesh from stl to obj, and update link in urdf file
sed 's/\.stl/\.obj/g' robot.urdf > robot_obj.urdf

