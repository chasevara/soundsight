# Project Description
The goal of soundsight is to visualize audible sound in human environments in insightful and visually compelling ways.\
\
LEDSoundSpectrum is an ambient sound visualizer that samples sounds in the environment and displays a real-time Fast Fourier Transform (FFT) though an RGB LED array. FFT frequency component magnitudes are mapped to a colorscale for LED output, giving the user the effect of a frequency "heat map" of their sound environment.\
\
***v0.2 is the most current working version with all below-listed features implemented***

## User Features
*(click images to link to feature demo videos on youtube)*

**FFT LED Display** - 128-light RGB LED array that displays the frequency spectrum spanning 1 Hz to 22050 Hz. Each light represents the magntiude of the correspinding discrete frequency component as a color value.

**User Settings Position LEDs** - 3 RGB LEDs that display the user-input potentiometer position a an RGB color-scale value. Uses the same colorscale as FFT LED Display.

**Brightness Control** - Manipulate a potentiometer to set brightness of output LEDs (Does not perfectly linearly scale colorscale hues since RGB settings are managed as discrete values. This is most noticeable at the lower range of this setting).

[![Brightness Control Demo](https://github.com/chasevara/soundsight/blob/main/documentation/videos/thumbnails/thumbnail_demo_brightness.jpg)](https://youtu.be/jOJQuSq8nRI)

**Threshold Control** - Manipulate a potentiometer to set the threshold (relative to the largest frequency component magnitude) below which a frequency component is not displayed. This allows the user to magnify dominant frequencies.

[![Magnitude Threshold Control Demo](https://github.com/chasevara/soundsight/blob/main/documentation/videos/thumbnails/thumbnail_demo_threshold.jpg)](https://youtu.be/jTJN3NtsjPg)

**Visual Persistence Control** - Manipulate a potentiometer to set the weight of new FFT data in the weighted moving average of FFT data that is displayed. Affects the visual persistence or "smoothness" of data output from the FFT LED display. Full weight setting results in approiximate display of the real-time sound environment, while reduced weight settings gives the user time to more easily visualize how the sound envronment changes through time.

[![Visual Persistence Control Demo](https://github.com/chasevara/soundsight/blob/main/documentation/videos/thumbnails/thumbnail_demo_persistence.jpg)](https://youtu.be/iq318SvnCOc)

**Colorscale Selection Button** - Press to alternate through a set of colorscale themes used to represent frequency component magnitudes on the FFT LED display.

[![Colorscale Selection Demo](https://github.com/chasevara/soundsight/blob/main/documentation/videos/thumbnails/thumbnail_demo_colorscale_change.jpg)](https://youtu.be/Ujx6axzXM5M)


## Hardware Description
**Driven by an Arduino Uno Rev 4 Wifi development board**\
Arm Cortex-M4 (48MHz)\
32kB RAM

Development board and LEDs powered by external 5V/10AMP DC source. A 1000mF capacitor is installed across mains to prevent power-up surges from damaging 5050 RGB LED components. User-input potentiometers arranged as voltage dividers.

**Hardware schematic:**

![](https://github.com/chasevara/soundsight/blob/main/documentation/schematics/schematic_soundsight.jpg)

## Software Description ##
The system is architected as a FreeRTOS scheduler within the Arduino environment, organized into four tasks:

**1. Setup:** Configures the board for I/O and enables MCU peripheral timers

**2. Supervisor:** Monitors user input to enact user settings. Debounces the user colorscale-select pushbutton.

**3. Audio:** Samples ambient audio; computes FFT; converts complex FFT data into real magnitude data; scales magnitude data in accordance with Parseval's signal energy theorum. The DC component of the FFT is not used.

**4. LED settings:** Maintains a moving weighted average of the FFT magnitude data, with weight setting from user input; Maps weighted average data to FFT LED Display, adjusting for user brightness, threshold, and colorscale set by the user. Each magnitude mapped to an LED is represented as an RGB value from the user-selected colorscale. Also drives the user potentiometer position LEDs

The setup task is halted after its single execution, while supervisor, audio, and LED tasks run concurrently and with different execution periods. FreeRTOS queues are used as the predominent synchronization solution.

**"LEDspectrum_config.h"** defines all macros used to configure the operations of the system. Includes debug-enable macros for debugging various parts of the system through serial output; Defines FreeRTOS stack and priority parameters.

**"colorscales.h"** defines colorscale RGB HEX value arrays. Used to drive RGB LED color settings 

***IMPORTANT:*** FreeRTOS heap capacity must be set to at least 3 kB in the board's FreeRTOS port configuratioin file.

## Open-Source Libraries Used: ##
FreeRTOS 11.0.1\
FastLED 3.6.0\
Arduino_CMSIS-DSP 5.7.0

## License ##
This project is licensed under the MIT License - see the [LICENSE.txt](https://github.com/chasevara/soundsight/blob/main/LICENSE.txt) file for details.
