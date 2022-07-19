# Electronic

## Bill of materials

| Component       |    Quantity     |
| :-------------: | :-------------: |
| Raspberry Pi 4  |        1        |
| [Raspberry Pi Camera](https://www.amazon.fr/gp/product/B07ZZ2K7WP/ref=ppx_yo_dt_b_asin_title_o00_s02?ie=UTF8&psc=1)  |        1        |
| [Speaker](https://www.amazon.fr/gp/product/B07RFQKY4R/ref=ppx_yo_dt_b_asin_title_o02_s01?ie=UTF8&psc=1)  |        1        |
| [Microphone](https://www.amazon.fr/gp/product/B01LCIGY8U/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)  |        1        |
| [3.2" HDMI LCD display (Waveshare)](https://www.waveshare.com/3.2inch-hdmi-lcd-h.htm) |        1        |
| [Nema 17 motor](https://www.amazon.fr/gp/product/B06XRFCP3X/ref=ppx_yo_dt_b_asin_title_o01_s01?ie=UTF8&th=1)   |        2        |
| [A4988 driver](https://www.amazon.fr/gp/product/B07MXXL2KW/ref=ppx_yo_dt_b_asin_title_o01_s01?ie=UTF8&psc=1)    |        2        |
| Capacitor ($100\mu F$)    |        2        |

## Circuit schematic

![Wiring diagram](https://github.com/RomainMaure/PixelBot/blob/main/electronic/wiring_diagram.png)

## Motor driver setting

Before powering the motors for the first time, the current limit of the drivers needs to be adjusted. Use the potentiometer of the driver for this purpose. You can follow this nice [tutorial](https://www.youtube.com/watch?v=7spK_BkMJys&t=735s) to see how it can be achieved. In my case, I set a reference voltage of 0.7V on both drivers.
