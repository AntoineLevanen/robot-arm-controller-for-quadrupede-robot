#Convert all mesh from stl to obj, and update link in urdf file
for f in *.stl; do
    ctmconv ${f} ${f%.stl}.obj
done
sed 's/\.stl/\.obj/g' robot.urdf > robot_obj.urdf

