import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

"""
Get the robot frames name
"""

sassa = RobotWrapper.BuildFromURDF("../sassa/robot.urdf", "../sassa/", pin.JointModelFreeFlyer())

# print(sassa.model.existFrame)
sassa.model.createData()
print(sassa.model.getFrameId('gripperright_sasm'))