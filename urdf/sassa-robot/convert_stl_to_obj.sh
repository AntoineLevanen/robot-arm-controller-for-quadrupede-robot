for f in *.stl; do
	ctmconv ${f} ${f%.stl}.obj
done

sed 's/\.stl/\.obj/g' robot.urdf > robot_obj.urdf
