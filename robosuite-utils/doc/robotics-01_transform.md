# Introduction to Robotics

[Introduction to Robotics book PDF](http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040), Third Edition.  *John J*. *Craig*,

[![Youtube Badge](https://img.shields.io/badge/Youtube-ff0000?style=flat-square&logo=youtube&link=https://www.youtube.com/channel/UCl-b60XKl2zxvBv3uiVvvbw)](https://www.youtube.com/channel/UCl-b60XKl2zxvBv3uiVvvbw) : 고려대학교 송재복 교수님

## Chapter 1. Spatial descriptions and Transformations

### 1-1. Descriptions: Pose, Position, Orientation

---

|                                                         |                                                          |
| :-----------------------------------------------------: | :------------------------------------------------------: |
| <img src= "../img/robotics-01_img1.png" height="250" /> | <img src= "../img/robotics-01_img18.png" height="250" /> |

---

#### - Pose = position(위치) + orientation(방위, 자세, ...)

- example : [ROS Pose Message](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)

#### - Position

- **Description of a position**

  - <a href="https://www.codecogs.com/eqnedit.php?latex=^{A}\textrm{\textit{P}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?^{A}\textrm{\textit{P}}" title="^{A}\textrm{\textit{P}}" /></a> : position vector of P written in {A}

  ![image-20210703222058469](../img/robotics-01_img2.png)

<div style="page-break-after: always;"></div>

#### - Orientation

- **Description of a orientation**

  ![image-20210703222058469](../img/robotics-01_img6.png)

  ---

  ![image-20210703222058469](../img/robotics-01_img9.png)

  ---

  - <a href="https://www.codecogs.com/eqnedit.php?latex=^{A}\hat{X}_{B}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?^{A}\hat{X}_{B}" title="^{A}\hat{X}_{B}" /></a> : the vector whose components are the projections of that vector onto the unit directions of its reference frame.

  ![image-20210703222058469](../img/robotics-01_img3.png)

  ![image-20210703222058469](../img/robotics-01_img4.png)

- **Physical meaning of rotation matrix**

  - <a href="https://www.codecogs.com/eqnedit.php?latex=^{A}\textrm{R}_{B}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?^{A}\textrm{R}_{B}" title="^{A}\textrm{R}_{B}" /></a> : rotation matrix from the coordinates of P relative to {B} to coordinates relative to {A}

  ![image-20210703222058469](../img/robotics-01_img5.png)

  <div style="page-break-after: always;"></div>

  > Example 

  ---

  ![image-20210703222058469](../img/robotics-01_img7.png)

  ---

  Given, ![image-20210703222058469](../img/latex3.gif)

  we obtain

  ![image-20210703222058469](../img/latex2.gif)

  we calculate <a href="https://www.codecogs.com/eqnedit.php?latex=^{A}\textrm{\textit{P}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?^{A}\textrm{\textit{P}}" title="^{A}\textrm{\textit{P}}" /></a> as

  ![image-20210703222058469](../img/latex4.gif)

  ---

- **Properties of a rotation matrix**

  **orthonormal matrix(직교 행렬)**

  ![image-20210703222058469](../img/robotics-01_img8.png)

  <img src="https://latex.codecogs.com/gif.latex?\begin{align*}^{A}{P}=^A{R}_{B}{}^{B}{P}\\^{B}{P}=^B{R}_{A}{}^{A}{P}&space;\end{align}" title="\begin{align*}^{A}{P}=^A{R}_{B}{}^{B}{P}\\^{B}{P}=^B{R}_{A}{}^{A}{P}&space;\end{align}" />

  <img src="https://latex.codecogs.com/gif.latex?^{A}{P}=^A{R}_{B}{}^B{R}_{A}{}^A{P}" title="^{A}{P}=^A{R}_{B}{}^B{R}_{A}{}^A{P}" />

  <img src="https://latex.codecogs.com/gif.latex?^A{R}_{B}{}^B{R}_{A}=I" title="^A{R}_{B}{}^B{R}_{A}=I" />

  Indeed, from linear algebra, we know that the inverse of a matrix with orthonormal columns is equal to its transpose.

  <div style="page-break-after: always;"></div>

  - Orthogonality conditions 

    ![image-20210703222058469](../img/latex5.gif)

  - Unit length conditions

    ![image-20210703222058469](../img/latex6.gif)

  Only 3 components are independent among 9 components of a rotation matrix because 6 conditions are imposed.]

  즉, 3개의 파라미터만 결정을 하면 orientation이 정해진다는 의미입니다.



### 1-2.  Operators: Translation, Rotation, and Transformation

![image-20210703222058469](../img/robotics-01_img10.png)

![image-20210703222058469](../img/robotics-01_img11.png)

![image-20210703222058469](../img/robotics-01_img12.png)

- Homogeneous transform (동차변환) 

  4 x 4 matrix

![image-20210703222058469](../img/latex7.gif)

- Pure translation

![image-20210703222058469](../img/latex11.gif)



- Pure Rotation (Elementary rotation)

  - General rotation : a suitable sequence of three elementary rotations on condition that two successive rotations are not made about parallel axes.

  - Fixed angle representation (12 sets)

    Rotation with respect to the fixed axes 👉 Absolute transforms

    Typically, XYZ fixed angles (roll, pitch, yaw angles)

  - [Euler angle](https://en.wikipedia.org/wiki/Euler_angles) representation (12 sets)

    Rotation with respect to the current axes (or body-fixed axes) 👉 Relative transforms

    XYZ, ZXY, XZX, XYX, YXZ, YZX, YXY, YZY, ZXY, ZYZ, ZXZ, ZYX

    Typically, ZYX Euler angles and ZYZ Euler angles

  - 각도 계산 시 [arctan2](https://numpy.org/doc/stable/reference/generated/numpy.arctan2.html)를 이용

    arctan : range [-pi/2, pi/2]

    arctan2 : range [-pi, pi], arctan2(y, x) = arctan2(sin(theta), cos(theta))

    ~~~python
    result_arctan = np.arctan([-1/-1]) * 180 / np.pi
    >>> [45.]
    result_arctan2 = np.arctan2([-1], [-1]) * 180 / np.pi
    >>> [-135.]
    ~~~

    

![ ](../img/robotics-01_img13.png)

| ![image-20210703222058469](../img/latex8.gif) | ![image-20210703222058469](../img/latex10.gif) | ![image-20210703222058469](../img/latex9.gif) |
| --------------------------------------------- | ---------------------------------------------- | --------------------------------------------- |

<div style="page-break-after: always;"></div>

- Homogeneous transform

  H = Translation x Rotation, 

  H = TR

  ![image-20210703222058469](../img/latex12.gif)
  
  <img src="https://latex.codecogs.com/gif.latex?^A{T}^{-1}_{B}" title="^A{T}^{-1}_{B}" />: Inverse  transform matrix

![image-20210703222058469](../img/latex13.gif)

![image-20210703222058469](../img/robotics-01_img15.png)

<img src="https://latex.codecogs.com/gif.latex?^A{T}^{-1}_{B}&space;=&space;{}^{B}_{A}{T}" title="^A{T}^{-1}_{B} = {}^{B}_{A}{T}" />

<div style="page-break-after: always;"></div>

### 1-3.  Relative transform and Absolute transform

- Relative transform
  - Translation relative to the current frame ==> Rotation relative to the current frame
  - Each transform is performed relative to the current frame.
  - The final frame is computed by **post-multiplying** (left -> right) the corresponding transformation matrices successively.
- Absolute transform
  - Rotation relative to the fixed frame -> Translation relative to the fixed frame
  - Each transform is performed relative to the fixed(reference) frame.
  - The final frame is computed by **pre-multiplying** (right -> left) the corresponding transformation matrices successively.

![image-20210703222058469](../img/robotics-01_img16.png)

![image-20210703222058469](../img/robotics-01_img17.png)



>H = T1 * R1 * T2 * R2
>
>Relative transform
>
>- (Translation of A -> B) ---> (Rotation of A -> B) ---> (Translation of B -> C) ---> (Rotation of B->C)
>
>Absolute transform
>
>- (Translation of A -> B) <--- (Rotation of A -> B) <--- (Translation of B -> C) <--- (Rotation of B->C)

<div style="page-break-after: always;"></div>

### 1-4. Transform Equations

> Examples

![image-20210703222058469](../img/robotics-01_img18.png)

![image-20210703222058469](../img/robotics-01_img19.png)

<div style="page-break-after: always;"></div>

### 1-5. More on representation of Orientation

#### - Inverse XYZ Fixed angles, ZYX Euler angles, ZYZ Euler angles problem

- **XYZ fixed angles**

  ![image-20210703222058469](../img/robotics-01_img20.png)

  ![image-20210703222058469](../img/robotics-01_img21.png)

  ![image-20210703222058469](../img/robotics-01_img22.png)

  ![image-20210703222058469](../img/robotics-01_img23.png)

  - if <img src="https://latex.codecogs.com/gif.latex?cos(\beta)&space;\neq&space;0" title="cos(\beta) \neq 0" />

    ![image-20210703222058469](../img/robotics-01_img24.png)

    **we always compute the single solution for which <img src="https://latex.codecogs.com/gif.latex?-90.0^{\circ}&space;\leq&space;\beta&space;\leq&space;90.0^{\circ}" title="-90.0^{\circ} \leq \beta \leq 90.0^{\circ}" />**

  - If <img src="https://latex.codecogs.com/gif.latex?\beta&space;=&space;90.0^{\circ}" title="\beta = 90.0^{\circ}" />

    <img src="https://latex.codecogs.com/gif.latex?\alpha&space;-&space;\gamma&space;=&space;atan2(-r_{12},&space;r_{22})" title="\alpha - \gamma = atan2(-r_{12}, r_{22})" />

    ![image-20210703222058469](../img/robotics-01_img25.png)

    

  - If <img src="https://latex.codecogs.com/gif.latex?\beta&space;=&space;-90.0^{\circ}" title="\beta = -90.0^{\circ}" />

    <img src="https://latex.codecogs.com/gif.latex?\alpha&space;&plus;&space;\gamma&space;=&space;atan2(-r_{12},&space;r_{22})" title="\alpha + \gamma = atan2(-r_{12}, r_{22})" />

    ![image-20210703222058469](../img/robotics-01_img26.png)

    

    In those cases, only the sum or the difference of <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> and <img src="https://latex.codecogs.com/gif.latex?\gamma" title="\gamma" /> can be computed.

    -> **Infinitely many solution for <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> and <img src="https://latex.codecogs.com/gif.latex?\gamma" title="\gamma" />**

    Not able to distinguish the rotation <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> and <img src="https://latex.codecogs.com/gif.latex?\gamma" title="\gamma" /> 

    ->**Representation singularity** (1 자유도 상실)

- **ZYX Euler angles**

  ![image-20210703222058469](../img/robotics-01_img27.png)

  **three rotations taken about fixed axes yield the same final orientation as the same three rotations taken in opposite order about the axes of the moving frame.**

- **ZYZ Euler angles**

  **we always compute the single solution for which <img src="https://latex.codecogs.com/gif.latex?0^{\circ}&space;\leq&space;\beta&space;\leq&space;180.0^{\circ}" title="0^{\circ} \leq \beta \leq 180.0^{\circ}" />**

  If <img src="https://latex.codecogs.com/gif.latex?\beta&space;=&space;0.0" title="\beta = 0.0" />or <img src="https://latex.codecogs.com/gif.latex?\beta&space;=&space;180.0" title="\beta = 180.0" />,

  In those cases, only the sum or the difference of <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> and <img src="https://latex.codecogs.com/gif.latex?\gamma" title="\gamma" /> can be computed.

  Not able to distinguish the rotation <img src="https://latex.codecogs.com/gif.latex?\alpha" title="\alpha" /> and <img src="https://latex.codecogs.com/gif.latex?\gamma" title="\gamma" /> 

  ->**Representation singularity** (1 자유도 상실)

<div style="page-break-after: always;"></div>

### 1-6. Equivalent angle-axis representation

[wikipedia](https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation)

![ ](../img/robotics-01_img13.png)

![ ](../img/robotics-01_img28.png)

![ ](../img/robotics-01_img29.png)

![ ](../img/robotics-01_img31.png)

**Singularity : The solution given by fails if <img src="https://latex.codecogs.com/gif.latex?\theta" title="\theta" /> = 0° or <img src="https://latex.codecogs.com/gif.latex?\theta" title="\theta" /> = 180**

<div style="page-break-after: always;"></div>

### 1-7. Unit Quaternion

[wikipedia](https://en.wikipedia.org/wiki/Quaternion)

- **Euler parameters.**

  ![ ](../img/robotics-01_img32.png)

  ![ ](../img/robotics-01_img33.png), Only three parameters are independent.

  ![ ](../img/robotics-01_img34.png)

  ![ ](../img/robotics-01_img35.png)

  **not Singularity**

---

<div style="text-align: right">next 👉  <a href="robotics-02_forward_kinematics.md"> forward kinematics</a> </div>

---

