# PicoSPIDisplay

Development of helper classes and integration of a SSD1351 display. This is a SPI display.
I know there are a lot of feature rich drivers for this. Most of them use buffering or double buffering. I don't want to implement a graphics library.
This implementation's goal is to get all out of the Pico MCU with the least overhead in CPU load and memory overhead.
SPI may be replaced with PIO in the future or be an additional option.
Currently trying to implement scaling/rotating an image with the interpolator. Still don't know whether this is completely possible.
