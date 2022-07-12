# Electronic

## Bill of materials

| Component       |    Quantity     |
| :-------------: | :-------------: |
| Raspberry Pi 4  |        1        |
| [3.2" HDMI LCD display (Waveshare)](https://www.waveshare.com/3.2inch-hdmi-lcd-h.htm) |        1        |
| Nema 17 motor   |        2        |
| A4988 driver    |        2        |
| Capacitor ($100\mu F$)    |        2        |

## Circuit schematic

![Wiring diagram](https://github.com/RomainMaure/PixelBot/blob/main/electronic/wiring_diagram.png)

## Motor driver setting

Before powering the motors for the first time, the current limit of the drivers needs to be adjusted. Use the potentiometer of the driver for this purpose. You can follow this nice [tutorial](https://www.youtube.com/watch?v=7spK_BkMJys&t=735s) to see how it can be achieved. In my case, I set a reference voltage of 0.7V on both drivers.
