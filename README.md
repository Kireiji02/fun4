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

- build the project
    ```bash
    cd
    cd workspace/
    colcon build
    source install/setup.bash
    ```

## Usage

