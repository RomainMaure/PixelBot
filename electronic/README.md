# Electronic

## Bill of materials

| Component       |    Quantity     |
| :-------------: | :-------------: |
| Raspberry Pi 4  |        1        |
| Raspberry Pi power supply (5V - 3A)  |        1        |
| [Raspberry Pi Camera](https://www.amazon.fr/gp/product/B07ZZ2K7WP/ref=ppx_yo_dt_b_asin_title_o00_s02?ie=UTF8&psc=1)  |        1        |
| [Speaker](https://www.amazon.fr/gp/product/B07RFQKY4R/ref=ppx_yo_dt_b_asin_title_o02_s01?ie=UTF8&psc=1)  |        1        |
| [Microphone](https://www.amazon.fr/gp/product/B01LCIGY8U/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)  |        1        |
| [3.2" HDMI LCD display (Waveshare)](https://www.waveshare.com/3.2inch-hdmi-lcd-h.htm) |        1        |
| [Motor power supply (12V - 2A)](https://www.amazon.fr/gp/product/B01G0Q3RWU/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1) |        1        |
| [Nema 17 motor](https://www.amazon.fr/gp/product/B06XRFCP3X/ref=ppx_yo_dt_b_asin_title_o01_s01?ie=UTF8&th=1)   |        2        |
| [A4988 driver](https://www.amazon.fr/gp/product/B07MXXL2KW/ref=ppx_yo_dt_b_asin_title_o01_s01?ie=UTF8&psc=1)    |        2        |
| [Capacitor ($100\mu F$)](https://www.lcsc.com/product-detail/Aluminum-Electrolytic-Capacitors-Leaded_LCSC-100uF-35V-6-3-11_C45076.html)    |        2        |

## Circuit schematic (using a breadboard)

![Wiring diagram](https://github.com/RomainMaure/PixelBot/blob/main/electronic/wiring_diagram.png)

## Circuit schematic (using the PixelBot pcb)

To avoid all the wiring, it is also possible to use the Raspberry pi hat specifically designed for PixelBot. I personally used JLCPCB to manufacture it from the [gerber file](https://github.com/RomainMaure/PixelBot/blob/main/electronic/Gerber_pixel_bot_pi_hat_V1_0). The following additional components will be needed for the pcb soldering:

| Component       |    Quantity     |
| :-------------: | :-------------: |
| [Power Jack 6.3mm](https://www.lcsc.com/product-detail/AC-DC-Power-Connectors_SOFNG-DC005-T20_C111567.html)  |        1        |
| [20-pin female header, P=2.54](https://www.lcsc.com/product-detail/Female-Headers_ZHOURI-2-54-2-20_C2977589.html)  |        1        |
| [4-pin male header, P=2.54mm](https://www.lcsc.com/product-detail/Pin-Headers_JST-Sales-America-RE-H042TD-1190-LF-SN_C265319.html)  |        2        |

## Motor driver setting

Before powering the motors for the first time, the current limit of the drivers needs to be adjusted. Use the potentiometer of the driver for this purpose. You can follow this nice [tutorial](https://www.youtube.com/watch?v=7spK_BkMJys&t=735s) to see how it can be achieved. In my case, I set a reference voltage of 0.7V on both drivers.
