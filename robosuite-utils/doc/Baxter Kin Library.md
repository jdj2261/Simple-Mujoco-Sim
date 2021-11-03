# Baxter's Kinematics Library Description



## 디렉토리 구조

- asset

  Directory of Robot Models

  - mjcf - mujoco xml 
  - urdf - urdf xml 

- kin_utils

  Directory of Kinematics

  - parsing
    - urdf_parser.py
    - urdf_tree.py
  - geometry (link, joint, frame)
    - frame.py
  - kinematics
    - transformation.py
    - kinematics.py
  - baxter.py

- utils

  - plot.py



## 이번 주 개발 내역

1. URDF (Unified Robot Description Format) 파싱
   - 참고자료
     - [ros wiki/urdf](http://wiki.ros.org/urdf/XML)
     - [ros/urdf_parser_py](https://github.com/ros/urdf_parser_py)
     - [python xml 파서 라이브러리](https://docs.python.org/ko/3/library/xml.etree.elementtree.html)
2. Link, Joint, Frame, Tree 작성
   - name, position, orientation, homogeneous, pose
   - joint parent, joint child
3. transformation 관련 함수 작성



## TO DO

- kinematics 
  - Forward
  - Inverse
- plot
  - 3D axis

