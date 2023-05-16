# Source code

## Raspberry Pi setup

To setup the Raspberry Pi, you will need a microSD card (32 GB recommended) and an SD card reader. To flash the operating system on the microSD card, the use of [Raspberry Pi Imager](https://www.raspberrypi.com/software/) is advised.

### Installation

On Raspberry Pi imager, select the *Other general-purpose OS (other)* option. Then choose *Ubuntu* and select *Ubuntu Desktop 22.04.2 LTS (64-bit)* and write it on your microSD card. After the writing operation, you can plug the microSD card on your Raspberry Pi and turn it on. 

After the setup process of Ubuntu, you will have to download the dependencies required to run the activity. To do so, you will first have to download the [PixelBot repository](https://github.com/RomainMaure/PixelBot) eitheir using git or by downloading the zip version of the repository. It is important that the PixelBot folder is located in your *home* directory. You can then install the required dependencies by using the script [install.sh](https://github.com/RomainMaure/PixelBot/blob/main/src/install.sh). To do so, open a terminal in the PixelBot/src/ folder and run:

```
chmod +x install.sh
./install.sh
```

### Possible additional steps

- You might have to rotate the screen on the LCD display, to do so, you can open a terminal and run one of the following commands:
    ```
    # To rotate 90 degrees on the right
    DISPLAY=:0 xrandr --output HDMI-1 --rotate right

    # To rotate 90 degrees on the left
    DISPLAY=:0 xrandr --output HDMI-1 --rotate left

    # To rotate 180 degrees
    DISPLAY=:0 xrandr --output HDMI-1 --rotate inverted

    # To reset the rotation back to normal
    DISPLAY=:0 xrandr --output HDMI-1 --rotate normal
    ```
    For more documentation, check the following [link](https://linuxhint.com/rotate-screen-in-raspberry-pi/).

## Testing scripts

Before running the robot interaction, make sure that each part of the robot is working properly (motors, speaker, LCD, buttons).

To do so, you can run individually each script located in the [test scripts](https://github.com/RomainMaure/PixelBot/tree/main/src/test_scripts) folder:

```
python3 <python_script_name>
```

## ROS2 packages

TODO

## Build

TODO

## Run

TODO

## Troubleshooting

TODO
