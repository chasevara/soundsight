**Soundsight is ambient sound spectrum visualizer**

Hardware components:

-Arduino Uno Rev 4 Wifi development board (Renesas RA4M1 MCU powered by Arm Cortex-M4)\
-WS2812b RGB LED strip (128x 5050 SMD addressable RGB LED devices)\
-KY-030 microphone part\
-10kOhm potentiometers for analog user inputs\
-5V/10A external powersupply with 1000mF capacitor to protect WS2812b on powerup

Software attributes:

-Open-source libraries used: FastLED, FreeRTOS for Arduino, Arm CMSIS DSP
 
There are four primary user inputs:
1. Collective LED brightness setting
2. Frequency relative magnitude threshold for display
3. Visual persistence of LED output (adjusts weight for moving average time decimation)
4. Magnitude colorcale theme selection

Additionally, there is a digital rotary-encoder input for tunning the system's audio sampling rate





