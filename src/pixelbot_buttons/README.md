# PixelBot Buttons

## Overview

This package allows to publish the state of PixelBot's buttons. 

**Keywords:** human-robot interaction, buttons

### License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The whole package is under MIT License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure@epfl.ch**

The [pixelbot_buttons](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_buttons) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [gpiozero](https://pypi.org/project/gpiozero/) for accessing the different pins of the Raspberry Pi.
    ```
	sudo pip3 install gpiozero
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_buttons
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_buttons pixelbot_buttons_node
```

## Nodes

### pixelbot_buttons_node

Publish the state of PixelBot's buttons.

#### Published topics

* **`/buttons_state`** ([std_msgs/UInt8MultiArray])

	List containing the state of each button of PixelBot ([right_button.is_pressed, left_button.is_pressed])


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).