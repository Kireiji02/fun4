# fun4-Robot Control in Cartesian Space

This project demonstrates how to control a robot's end-effector in Cartesian space (x, y, z) using `teleop_twist_keyboard` and inverse kinematics. The robot's joints are controlled indirectly through the Jacobian matrix, which converts Cartesian velocities into joint velocities.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Inverse Kinematics Explanation](#inverse-kinematics-explanation)
- [Files](#files)
- [Known Issues](#known-issues)
- [Contributing](#contributing)
- [License](#license)

## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Humble** (or a compatible version)
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

- install these following packages:
    ```bash
    pip install numpy spatialmath-python roboticstoolbox-python
    ```

- make sure numpy version is compatible
    ```bash
    pip show numpy
    pip install numpy==1.23.3
    ```

- ROS2 programs
    ```bash
    sudo apt install ros-humble-desktop-full
    ```

### Workspace

- prepare your workspace as empty folder
    ```bash
    cd
    mkdir workspace
    cd workspace/
    mkdir src
    ```

- go to src and clone this github repository as ros2 package
    ```bash
    cd
    cd workspace/src/
    git clone https://github.com/Kireiji02/fun4
    ```

- if you don't have colcon:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

- build the project
    ```bash
    cd
    cd workspace/
    colcon build
    source install/setup.bash
    ```

## Usage

- run the launch file containing controller.py, randomizer.py and pose_analyzer.py
    ```bash
    ros2 launch fun4 fun.launch.py
    ```

- open new terminal (ctrl + alt + t) and run keyboard_teleop.py
    ```bash
    ros2 run fun4 keyboard_teleop.py
    ```

- open another terminal for teleop_twist_keyboard
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard 
    ```
    
## Features

### Every Mode can be controlled by this node
![keyboard_teleop](images/keyboard_teleop.png)
keyboard_teleop.py - to start navigate to node's terminal and press any number keys

### Mode1: Inverse Position Kinematics (IPK)
by pressing '1' key on your keyboard Mode1 will be selected but not yet confirmed

- press tab to start input target coordinate
- input x,y,z coordinate pressing tab after each one
- press tab again and node will display "value ready : [0.0, 0.0, 0.0], press c"
- press c to confirm mode selection

note: if input value was wrong press "r" key to reset

### Mode2: Teleoperation
press key '2' in keyboard_teleop node, press 'c' key to confirm selection and navigate to teleop_twist_keyboard node:

i: move forward (x+)
,: move backward (x-)
shift + j: move left (y+)
shift + l: move right (y-)
t: move up (z+)
b: move down (z-)

### Mode3: Automatic
press key '3' in keyboard_teleop node and press 'c' key to confirm selection.

## Inverse Kinematics Explanation

The robot's movement is controlled in Cartesian space by calculating joint velocities using inverse kinematics through the Jacobian matrix. The Jacobian describes how small changes in joint angles affect the position and orientation of the end-effector in space.

Here’s a simplified breakdown of the process:

1. The desired Cartesian velocity (v) is input via teleop_twist_keyboard.
2. The Jacobian matrix (J) is calculated at the robot’s current configuration.
3. The pseudo-inverse of the Jacobian (J⁺) is used to compute the necessary joint velocities (q_dot).
4. The joint velocities (q_dot) are applied to the robot, causing the joints to move in a way that moves the end-effector in Cartesian space.
Key Formula:

𝑞_𝑑𝑜𝑡=𝐽⁺⋅𝑣

Where J⁺ is the pseudo-inverse of the Jacobian and v is the desired Cartesian velocity.