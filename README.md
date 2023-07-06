# robot-arm-controller-for-quadrupede-robot
Kinematic controller for a 4 degree of freedom robot arm mounted on a quadrupede robot


---

The goal of this project is to simulate a small robot arm with four dof on the SASSA quadrupede robot *(insert link here)* and see the effect it as on the robot.

This repo contains :  
* the robot URDF file
* other URDF file for visual purpose (plane, sphere, ...)
* Python programme to to automatically generate URDF file from the OnShape CAD file *(insert link here)* and reduce stl file size
* Python program to visualize the robot arm work space
* the robottic controller

---

The robot arm must carry a camera and a mouth like gripper on the end effector.

The robot is controlled using a second order inverse kinematic with the aim of implementing  