# robot-arm-controller-for-quadrupede-robot
Kinematic controller for a 4 degrees of freedom robot arm mounted on a quadruped robot


---

The goal of this project is to design and simulate a small robot arm with four DoF on the SASSA quadruped robot https://github.com/Gepetto/sassa and see the effect it as on the robot.

This repository contains :  
* the robot's URDF file
* other URDF file for visual purpose (plane, sphere, ...)
* Python programming to automatically generate an URDF file from the OnShape CAD file https://cad.onshape.com/documents/061760026d204032b779c261/w/af85f6d1bc175794b599d7d2/e/b15b2161b253b35f334237dc?renderMode=0&uiState=64c7cc15abc8a43086274648 and reduce STL mesh file size
* the robotic controllers
* Scenarios with which the robot can be tested and compared with other configurations

---

The robot arm carries a camera and a mouth like gripper on the end effector.

The robot is controlled using a second order inverse kinematic with a close loop.
