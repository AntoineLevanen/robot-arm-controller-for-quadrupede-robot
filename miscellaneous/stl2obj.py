import os
from stl_obj_convertor import convertor

def stl2Obj(file_path):
    # To get only files present in a path
    list_1 = os.listdir(path=r"root/home/project")
    
    # Loop through each value in the list_2
    for val in list_1:
    
        # Remove the value from list_2 if the "." is
        # not present in value
        if "." not in val:
            list_1.remove(val)
    print(list_1)



if __name__ == "__main__":
    stl2Obj("/home/alevanen/Documents/StageM1/robot-arm-controller-for-quadrupede-robot/urdf/sassa-robot")