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
- `teleop_twist_keyboard` package:
    ```bash
    sudo apt install ros-humble-teleop-twist-keyboard
    ```

- install these following packages:
    ```bash
    pip install numpy spatialmath-python roboticstoolbox-python
    ```

- make sure numpy version is compatible
    ```bash
    pip3 install numpy==1.23.3
    ```
    
git clone https://github.com/your-username/robot-control-cartesian.git
cd robot-control-cartesian


