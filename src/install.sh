#!/usr/bin/env bash

echo "===Installing ROS2 Jazzy==="
cd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

echo "===Installing utilities==="
sudo apt install terminator

echo "===Installing dependencies==="
sudo apt install python3-pip
sudo apt install ros-jazzy-ament-index-python
sudo apt install python3-colcon-common-extensions
sudo apt install python3-pygame
sudo apt install python3-pyaudio
sudo apt install espeak
sudo apt install ffmpeg
sudo apt-get install flac
sudo apt-get install python3-scipy
sudo pip3 install numpy
sudo pip3 install pyttsx3
sudo pip3 install gTTS
sudo pip3 install playsound
sudo pip3 install pydub
sudo pip3 install adafruit-circuitpython-servokit
sudo pip3 install gpiozero
pip install lgpio
sudo apt-get install python3-rpi.gpio

echo "===Creating a ROS2 workspace==="
mkdir -p ~/ros2_ws/src

echo "===Copying PixelBot packages to ROS2 workspace==="
cp -a ~/PixelBot/src/pixelbot_msgs ~/ros2_ws/src/
cp -a ~/PixelBot/src/pixelbot_audio ~/ros2_ws/src/
cp -a ~/PixelBot/src/pixelbot_motors ~/ros2_ws/src/
cp -a ~/PixelBot/src/pixelbot_buttons ~/ros2_ws/src/
cp -a ~/PixelBot/src/pixelbot_display ~/ros2_ws/src/
cp -a ~/PixelBot/src/pixelbot_interaction ~/ros2_ws/src/

echo "===Building PixelBot packages==="
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo "===Automating sourcing of ROS2 and the previously created workspace==="
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "===PixelBot motors troubleshooting==="
sudo adduser $USER dialout
