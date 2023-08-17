# PixelBot Interaction

## Overview

This package allows to launch the robot-based learning activity aiming at raising awareness of gender inequality among children. 

**Keywords:**  human-robot interaction, robot-mediated storytelling activity, gender equality.

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

The whole package is under GPL-3.0 License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure21@gmail.com**

The [pixelbot_interaction](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_interaction) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) for the custom ROS2 headers.
- [pixelbot_audio](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_audio) to allow the robot to speak and play sounds.
- [pixelbot_display](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_display) to allow to perform facial expressions on PixelBot's LCD.
- [pixelbot_motors](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_motors) to control the motors of the robot.

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_interaction
    ```

## Usage

You can run the main node alongside the other PixelBot nodes with:
```
ros2 launch pixelbot_interaction pixelbot_interaction.launch.py
```

## Nodes

### pixelbot_interaction_node

This node depends on the other PixelBot packages and implements the main learning activity aiming at raising awareness of gender inequality among children.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
