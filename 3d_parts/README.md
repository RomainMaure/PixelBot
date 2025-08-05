# Mechanical design

## Bill of materials

### 3D prints

The table below describes the parts to be 3D printed to build the robot (all parts can be printed using PLA):

| Component       |    Quantity     |
| :-------------: | :-------------: |
| [Bottom plate](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/bottom_plate.stl) |        1        |
| [Leg](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/leg.stl) |        2        |
| [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl) |        1        |
| [Servo holder](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/servo_holder.stl) |        4        |
| [Arm](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/arm.stl)    |        2        |
| [Enclosure top](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure_top.stl)    |        1        |
| [Front head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/front_head.stl)    |        1        |
| [Back head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/back_head.stl)    |        1        |
| [Antenna](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/antenna.stl)    |        2        |

### Ironmongery

The table below describes the components needed to connect the parts with each other:

| Component       |    Quantity     |
| :-------------: | :-------------: |
| [M2x8 screw](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        12        |
| [M2x16 screw](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        8        |
| [M3x8 screw](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        13        |
| [M3x12 screw](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        6        |
| [M3x30 screw](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        6        |
| [M2 nut](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        8        |
| [M3 nut](https://www.amazon.fr/gp/product/B073SS7D8J/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)    |        17        |
| [M2 insert](https://www.amazon.fr/gp/product/B07CPRHP2X/ref=ppx_yo_dt_b_asin_title_o08_s02?ie=UTF8&psc=1)    |        8        |
| [M3 insert](https://www.amazon.fr/gp/product/B07CPRHP2X/ref=ppx_yo_dt_b_asin_title_o08_s02?ie=UTF8&psc=1)    |        8        |

## Assembly

The table below describes the order in which the robot is assembled and how each part is connected to the others:

| Parts connexion       |    Needed to connect the parts    |
| :-------------: | :-------------: |
| [Legs](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/leg.stl) to [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl) |        6 M3x12 screws and 6 M3 nuts        |
|  [Bottom plate](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/bottom_plate.stl) to [Legs](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/leg.stl)  |        6 M3x30 screws and 6 M3 nuts        |
| [Raspberry Pi 5](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/RASPBERRY_PI_5_cooler.stl) to [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl)    |        4 M2x8 screws and 4 M2 inserts         |
| [PCA9685](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/PCA9685.stl) to [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl)    |        4 M2x8 screws and 4 M2 inserts         |
| [Servos](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/TowerPro_SG90.stl) to [Servo holders](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/servo_holder.stl) and [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl)    |        4 M2x16 screws and 4 M2 nuts        |
| [Servos](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/TowerPro_SG90.stl) to [Servo holders](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/servo_holder.stl) and [Back head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/back_head.stl)    |        4 M2x16 screws and 4 M2 nuts        |
| [LCD](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/WaveShare_5_LCD.stl) to [Front head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/front_head.stl)   |     4 M3x8 screws and 4 M3 inserts          |
| [Front head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/front_head.stl) to [Enclosure top](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure_top.stl)    |       2 M3x8 screws and 2 M3 nuts         |
| [Back head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/back_head.stl) to [Enclosure top](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure_top.stl)    |    2 M3x8 screws and 2 M3 nuts            |
| [Front head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/front_head.stl) to [Back head](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/back_head.stl)    |        1 M3x8 screw and 1 M3 nuts        |
| [Enclosure top](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure_top.stl) to [Enclosure](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/enclosure.stl)    |        4 M3X8 screws and 4 M3 inserts        |
| [Arms](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/arm.stl) to [Servos](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/TowerPro_SG90.stl)    |        2 M2X8 screws        |
| [Antennae](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/antenna.stl) to [Servos](https://github.com/RomainMaure/PixelBot/blob/pixelbot_v2/3d_parts/stl/other/TowerPro_SG90.stl)    |        2 M2X8 screws        |

TODO: Video to describe the assembly of the robot.
