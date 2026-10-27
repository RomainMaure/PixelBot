# PixelBot Audio

## Overview

This package allows PixelBot to speak and play sounds. 

**Keywords:**  human-robot interaction, audio communication, voice synthesis

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

The whole package is under GPL-3.0 License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure21@gmail.com**

The [pixelbot_audio](https://github.com/RomainMaure/PixelBot/tree/pixelbot_v2/src/pixelbot_audio) package has been tested under [ROS2 Jazzy](https://docs.ros.org/en/humble/index.html) on Ubuntu 24.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [Piper](https://github.com/OHF-Voice/piper1-gpl): text to speech library for Python.
    ```
	sudo pip3 install piper-tts
    ```    
- [Playsound](https://pypi.org/project/playsound/), [Pydub](https://pypi.org/project/pydub/), [PyAudio](https://pypi.org/project/PyAudio/), [FFmpeg](https://ffmpeg.org/), [eSpeak](https://doc.ubuntu-fr.org/espeak) and [Flac](https://doc.ubuntu-fr.org/flac): a set of audio related libraries.
    ```
    sudo pip3 install playsound
    sudo pip3 install pydub
    sudo apt install python3-pyaudio
    sudo apt install ffmpeg
    sudo apt install espeak
    sudo apt-get install flac
    ```

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

#### Subscribed Topics

* **`/tts_input`** ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

	The text to be said by the TTS engine.

#### Published Topics

* **`/recognizer_is_active`** ([std_msgs/Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))

	The state of the speech recognition engine (activated/deactivated).

#### Services

* **`change_language`** ([pixelbot_msgs/SetLanguage](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/src/pixelbot_msgs/srv/SetLanguage.srv))

	Allows to specify the language to be spoken (french, english, or german):
    ```
    # Change to english
	ros2 service call /change_language pixelbot_msgs/SetLanguage "language: 'english'"
    ```
    ```
    # Change to french
	ros2 service call /change_language pixelbot_msgs/SetLanguage "language: 'french'"
    ```
    ```
    # Change to german
	ros2 service call /change_language pixelbot_msgs/SetLanguage "language: 'german'"
    ```

* **`play_sound`** ([pixelbot_msgs/PlaySound](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))

	Allows PixelBot to play a sound:
    ```
	ros2 service call /play_sound pixelbot_msgs/PlaySound "sound_name: 'happy.mp3'"
    ```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
