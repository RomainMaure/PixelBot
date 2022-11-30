# PixelBot Audio

## Overview

This package allows PixelBot to speak. 

**Keywords:**  human-robot interaction, audio communication

### License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The whole package is under MIT License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure@epfl.ch**

The [pixelbot_audio](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_audio) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) for the custom ROS2 headers.
- [Pygame](https://www.pygame.org/news) (Python game developping library) for the visual animation of the robot's emotions on its LCD.
    ```
	sudo apt install python3-pygame
    ```
    TO COMPLETE

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_audio
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_audio pixelbot_audio_node
```

## Nodes

### pixelbot_audio_node

Allows PixelBot to speak.

#### Services

TO COMPLETE

* **`display_emotion`** ([pixelbot_msgs/DisplayEmotion](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/DisplayEmotion.srv))

	Perform the desired emotion on PixelBot's LCD. For example:
    ```
	ros2 service call /display_emotion pixelbot_msgs/DisplayEmotion "desired_emotion: 'happy'"
    ```


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
