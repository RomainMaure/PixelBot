# Source code

## Raspberry Pi setup

To setup the Raspberry Pi, you will need a microSD card of at least 32 GB and an SD card reader. To flash the operating system on the microSD card, the use of [Raspberry Pi Imager](https://www.raspberrypi.com/software/) is advised.

### Installation using the PixelBot custom image

You can setup the Raspberry Pi by using the custom image made for PixelBot. In this image, all the required dependencies are already installed.

To do so, first download the image file (TODO: link to image file), then, on Raspberry Pi imager, select the *Use custom* option. Choose the previously downloaded image, select your storage device and click on *Write*. After the writing operation, you can plug the microSD card on your Raspberry Pi and you are ready to go.

### Installation from scratch

You can also install all the dependencies and required software by yourself.

To do so, on Raspberry Pi imager, select the *Raspberry Pi OS (other)* option. Then choose *Raspberry Pi OS (64-bit)* and write it on your microSD card. After the writing operation, you can plug the microSD card on your Raspberry Pi and start downloading the required dependencies. To do so, you can use the script `install.sh` by running in a terminal:

```
chmod +x install.sh
./install.sh
```

Note: The robot has been tested under Raspbian Bullseye. 

### Additional required steps

TODO

## Testing scripts

Before running the robot interaction, make sure that each part of the robot is working properly (the motors with the pan/tilt mechanism, the camera, the speaker, the microphone, the LCD display etc).

To do so, you can run individually each script located in the [test scripts](https://github.com/RomainMaure/PixelBot/tree/main/src/test_scripts) folder.

## ROS2 packages

TODO

## Build

TODO

## Run

TODO

## Troubleshooting

TODO
