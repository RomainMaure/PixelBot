#!/usr/bin/env bash

echo "===Installing ROS2 Humble==="
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

echo "===Setting up a ROS2 workspace==="
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
cd

echo "===Automating sourcing of ROS2 and the previously created workspace==="
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/pixelbot/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "===Installing utilities==="
sudo apt install terminator

echo "===Installing dependencies==="
sudo apt install python3-pip
sudo apt install ros-humble-ament-index-python
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

# tester si besoin de modifier file audio
# chmod +x necessary ?
