# PixelBot Motors

## Overview

This package allows to control the motors of PixelBot. 

**Keywords:** motors, gestures

### License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The whole package is under MIT License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure@epfl.ch**

The [pixelbot_motors](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_motors) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) for the custom ROS2 headers.
- [adafruit_servokit](https://pypi.org/project/adafruit-circuitpython-servokit/) to control multiple servo motors through a PCA9865.
    ```
	sudo pip3 install adafruit-circuitpython-servokit
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_motors
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_motors pixelbot_motors_node
```

## Nodes

### pixelbot_motors_node

Allows to control the motors of PixelBot.

#### Services

* **`motors_movement`** ([pixelbot_msgs/MotorsMovement](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/MotorsMovement.srv))

	Move the motors of your choice to a given angle. For example:
    ```
    # Move the right arm motor to an angle position of 180 degrees
	ros2 service call /motors_movement pixelbot_msgs/MotorsMovement "body_parts: ['right_arm']
    angles: [180]"
    ```
    ```
    # Move all the motors to a given angle position
	ros2 service call /motors_movement pixelbot_msgs/MotorsMovement "body_parts: ['right_arm', 'left_arm', 'right_antenna', 'left_antenna']
    angles: [180, 0, 90, 135]"
    ```


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).