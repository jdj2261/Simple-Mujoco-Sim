# Simple Mujoco Sim

This repository was created for manipulator control demo in a mujoco environment.

You can see four robot(`Franka Emika's panda`, `Kuka's iiwa14 and iiwa7`, `Rethink's sawyer`) demos.

As a library to solve kinematics, I used [`pykin`](https://github.com/jdj2261/pykin) python kinematics library.

I referenced it [franka_sim](https://github.com/vikashplus/franka_sim) and [sawyer_sim](https://github.com/vikashplus/sawyer_sim) to create a mujoco environment 
and [robosuite](https://github.com/ARISE-Initiative/robosuite) to control joint position, velocity, torque.

## Installation

### Requirements

You need a [mujoco-py](https://github.com/openai/mujoco-py) and [pykin](https://github.com/jdj2261/pykin.git) library.

I recommend to install using conda or venv

Demo is only available in mujoco version 2.0 or higher.

- Install Mujoco

  - for version 2.0
    - Download the MuJoCo version 2.0 binaries for [Linux](https://roboti.us/download/mujoco200_linux.zip) or [OSX](https://roboti.us/download/mujoco200_macos.zip).
    - Extract the downloaded `mujoco200` directory into `~/.mujoco/mujoco200`.
    - Copy License key `mjkey.txt` file to `~/.mujoco/mujoco200/bin/` and `~/.mujoco/`

  - for version 2.1
    - Download the MuJoCo version 2.1 binaries for [Linux](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz) or [OSX](https://mujoco.org/download/mujoco210-macos-x86_64.tar.gz).
    - Extract the downloaded `mujoco210` directory into `~/.mujoco/mujoco210`.

- Install Mujoco-py

  - in Mac 

    <details>
      <summary>Example</summary>

    ~~~
    $ brew install llvm
    $ brew install boost
    $ brew install hdf5
    
    # Add this to your .bashrc or .zshrc:
    export PATH="/usr/local/opt/llvm/bin:$PATH"
    
    export CC="/usr/local/opt/llvm/bin/clang"
    export CXX="/usr/local/opt/llvm/bin/clang++"
    export CXX11="/usr/local/opt/llvm/bin/clang++"
    export CXX14="/usr/local/opt/llvm/bin/clang++"
    export CXX17="/usr/local/opt/llvm/bin/clang++"
    export CXX1X="/usr/local/opt/llvm/bin/clang++"
    
    export LDFLAGS="-L/usr/local/opt/llvm/lib"
    export CPPFLAGS="-I/usr/local/opt/llvm/include"
    ~~~

    </details>

  - in Ubuntu 18.04 or 20.04

    <details>
      <summary>Example</summary>

    ~~~
    $ sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3
    
    # Add this to your .bashrc:
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/.mujoco/mujoco200/bin
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
    ~~~

    </details>

  - install mujoco-py using pip

    - for version 2.0
    
      ~~~
      $ pip install -U 'mujoco-py<2.1,>=2.0'
      ~~~
    
    - for version 2.1
    
      ~~~
      $ pip install -U 'mujoco-py<2.2,>=2.1'
      ~~~
    
      

- Install pykin

  You need a [fcl](https://github.com/flexible-collision-library/fcl.git) library to check collsion.

  - in mac

    <details>
      <summary>Example</summary>

    ~~~
    # install octomap
    $ git clone https://github.com/OctoMap/octomap.git
    $ cd octomap
    $ mkdir build && cd build
    $ cmake ..
    $ make
    $ make install
    
    # install fcl
    $ git clone https://github.com/flexible-collision-library/fcl.git
    $ cd fcl
    $ git checkout 0.5.0
    $ mkdir build && cd build
    $ cmake ..
    $ make
    $ make install
    ~~~

    </details>

  - in Ubuntu 18.04

    <details>
      <summary>Example</summary>

    ~~~
    $ sudo apt install liboctomap-dev
    $ sudo apt install libfcl-dev
    ~~~

  - install python-fcl and pykin using pip

    ~~~
    $ pip install python-fcl
    $ pip install pykin
    ~~~

### Install Simple-Mujoco-Sim

~~~
$ cd ~/
$ git clone https://github.com/jdj2261/Simple-Mujoco-Sim.git

## For using robot's kinematics and planning
$ git clone --recurse-submodules https://github.com/jdj2261/pykin.git
$ cd Simple-Mujoco-Sim
$ ln -s ~/pykin . 
~~~

## Demo

### 1. PID Tunning



### 2. Joint Position Control
