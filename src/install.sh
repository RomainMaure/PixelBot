#!/usr/bin/env bash

echo "===Installing ROS2 Humble==="
curl -O https://raw.githubusercontent.com/RomainMaure/rpi-bullseye-ros2/main/install.bash
bash install.bash humble aarch64 0.2.0 /opt/ros

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
sudo apt install python3-pyaudio
sudo apt install espeak
sudo apt-get install flac
sudo pip3 install opencv-python
sudo pip3 install pyttsx3
sudo pip3 install SpeechRecognition

# Activate legacy camera ????
# Test avec forked ros2 install -> should be ok now
# tester si besoin de modifier file audio
# chmod +x necessary ?
