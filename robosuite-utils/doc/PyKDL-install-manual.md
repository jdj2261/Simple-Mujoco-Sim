# PyKDL installation

**Ubuntu 18.04 환경에서 PyKDL 라이브러리 설치**

[orocos github](https://github.com/orocos/orocos_kinematics_dynamics) 참조하여 진행

## 1. orocos_kdl 설치

### Shared instructions

~~~
$ sudo apt update
$ sudo apt install libeigen3-dev libcppunit-dev
$ sudo apt install doxygen graphviz
~~~

### Compilation

~~~
$ git clone https://github.com/orocos/orocos_kinematics_dynamics.git
$ cd orocos_kdl
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
~~~



## 2. python_orocos_kdl 설치

python3.6 에서 설치 진행

~~~
$ sudo apt-get install python3-psutil python3-future
$ sudo apt-get install python3-sphinx
$ cd orocos_kdl
$ git submodule update --init
$ cd python_orocos_kdl
CMakeLists.txt 수정
16-18, 20번째 줄 주석처리 (python version과 관련되어 있음)
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo mkae install
bashrc에 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib 추가
$ sudo ldconfig
~~~

PyKDL.so 파일 python3.6 dist-packages 안으로 복사

~~~
$ cd python_orocos_kdl/build
$ sudo cp -r PyKDL.so /usr/local/lib/python3.6/dist-packages
~~~

python3에서 import PyKDL 되는지 확인
