# Introduction to Robotics

[Introduction to Robotics book PDF](http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040), Third Edition.  *John J*. *Craig*,

[![Youtube Badge](https://img.shields.io/badge/Youtube-ff0000?style=flat-square&logo=youtube&link=https://www.youtube.com/channel/UCl-b60XKl2zxvBv3uiVvvbw)](https://www.youtube.com/channel/UCl-b60XKl2zxvBv3uiVvvbw) : 고려대학교 송재복 교수님

## Chapter 2. Manipulator kinematics

### 2-1. Introduction

- **Kinematics**

  - The relationship between position, velocity, acceleration and time of the arm links.

  - The forces which cause motion are not included in kinematics.

- **Roles of kinematics in the robot arm**

  - The relationship between the end-effector pose(position, orientation) and the joint variables can be obtained by studying kinematics.

- **Joint space(관절 공간)**

  - Joint vector : a vector consisting of n joint variables of arm

    <img src="https://latex.codecogs.com/gif.latex?q&space;=&space;\begin{bmatrix}&space;q_{1}\\&space;\vdots&space;\\&space;q_{n}&space;\end{bmatrix},&space;where&space;\&space;q_{i}&space;:&space;joint&space;\&space;variable(i=1,...,n)" title="q = \begin{bmatrix} q_{1}\\ \vdots \\ q_{n} \end{bmatrix}, where \ q_{i} : joint \ variable(i=1,...,n)" />

  - Joint space : a space consisting of all joint vectors

- **Cartesian space (직교 공간) (or operational, task space)**

  - End- effector pose vector

    ![image-20210703222058469](../img/latex14.gif), where p : end-effector position & <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> : end-effector orientation

  - A space in which the end-effector pose is measured in the Cartesian coordinate system.

<img src= "../img/robotics-01_img36.png" height="300" />

<div style="page-break-after: always;"></div>

### 2-2. Forward kinematics

- **Joint space -> Cartesian space**

  <img src= "../img/robotics-01_img37.png" height="300" />

### 2-3. Inverse kinematics

- **Cartesian space -> Joint space**

|                                            |                                            |
| ------------------------------------------ | ------------------------------------------ |
| <img src= "../img/robotics-01_img38.png"/> | <img src= "../img/robotics-01_img39.png"/> |

---

<div style="text-align: right">next 👉  <a href="robotics-03_jacobian.md">jacobian matrix</a> </div>

---
