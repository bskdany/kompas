# kompas

Kompas is an portable, handleld hardware device that will point you to your most desired location: LCBO, Lazeez, E7, you name it

## Software

We use the Arduino language to interact with the breakout boards.
We recieve GPS location from connecting to satellites, constantly updating the precise direction of the target location.
The magnetometer is calibrated for hard-iron, and later calibrated to align with North, as well as the LEDS of the Kompas.
Caching and storage to FLASH memory allows for Kompas users to operate in a signal-less situation with their most recent GPS location, even on reboot of the system.


## Hardware

The project is composed of:
  - Raspberry pi pico 2w microcontroler
  - LIS3MDL magnetometer to sense magnetic field
  - DFRobot TEL015 GNSS module for global positioning
  - Pico SC1634 battery shim for battery power and charging
  - 1100mAh Lithium Polymer Ion Battery
  - 16-WS2813 Mini RGB LED Ring for the compass dial


All the components got soldered on a prototype board and put inside a custom enclosure
