# PixelBot Display

## Overview

This package allows to perform facial animations on PixelBot's LCD. 

**Keywords:** facial animation, human-robot interaction, Pygame



### License

The whole package is under MIT License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

This README is based on the project [ros_best_practices](https://github.com/leggedrobotics/ros_best_practices), Copyright 2015-2017, Péter Fankhauser. It is licensed under the BSD 3-Clause Clear License.

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure@epfl.ch**

The [pixelbot_display](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_display) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04 and Raspberry Pi OS Bullseye (64-bit).
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs]() (TODO: correct link) for the custom ROS2 headers.
- [Pygame](https://www.pygame.org/news) (Python game developping library) for the visual animation of the robot's emotions on its LCD.
    ```
	sudo apt install python3-pygame
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with ROS:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_display
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_display pixelbot_display_node
```

## Nodes

### pixelbot_display_node

Display facial animations on the robot's LCD.

#### Services

TODO correct link to message definition

* **`display_emotion`** ([pixelbot_msgs/DisplayEmotion](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

	Perform the desired emotion on PixelBot's LCD. For example:
    ```
	ros2 service call /display_emotion pixelbot_msgs/DisplayEmotion "desired_emotion: 'happy'"
    ```


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
