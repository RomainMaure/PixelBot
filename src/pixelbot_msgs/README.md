# PixelBot ROS2 Headers

## Overview

This package contains the custom message and service headers for the PixelBot learning activity, to mediate the communication between its ROS packages. 

### License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The whole package is under MIT License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure@epfl.ch**

The [pixelbot_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04 and Raspberry Pi OS Bullseye (64-bit).
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws/src
    colcon build --packages-select pixelbot_msgs
    ```