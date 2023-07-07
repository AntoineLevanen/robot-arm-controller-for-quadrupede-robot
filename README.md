# robot-arm-controller-for-quadrupede-robot
Kinematic controller for a 4 degrees of freedom robot arm mounted on a quadrupede robot


---

The goal of this project is to simulate a small robot arm with four DoF on the SASSA quadrupede robot *(insert link here)* and see the effect it as on the robot.

This repo contain :  
* the robot URDF file
* other URDF file for visual purpose (plane, sphere, ...)
* Python programme to to automatically generate URDF file from the OnShape CAD file *(insert link here)* and reduce meshes file size
* the robotic controllers

---

The robot arm will carry a camera and a mouth like gripper on the end effector.

The robot is controlled using a second order inverse kinematic.
