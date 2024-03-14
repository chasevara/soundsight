/**
 * @file WS2812b_colorscale.h
 * @brief Functions to drive a WS2812b RGB LED array with HEX color values mapped
 * to an input array of caller-input doubles. Wrutten for Arduino Uno R4 Wifi.
 * Each RGB LED "i" is set to the relative strength of the input array["i"].
 * @copyright Chase Vara, 2024
 **/
#ifndef WS2812B_COLORSCALE_H_
#define WS2812B_COLORSCALE_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <FastLED.h>

#define SET_LEDS_OFF CRGB(0, 0, 0)

/**
 * @fn LED_colorscale_set32
 * @brief Function that maps input value array to color scale and updates the
 * WS2812b settings to represent the resultant colorscale (32 color values)
 * @param vals Array of value to map to colorscale settings
 * @param leds Array of current LED array settings
 * @param palette_ptr Pointer to FastLED color palette object
 * @param num_leds The number of serial 5050SMD LED units to drive in the array
 * @param max_palette_index max index of CRGGM color palette for color scale
 * @param data_LED_scale the number of LEDs to represent each val in vals array
 * @param mask_low_fraction The normal fraction (0.0 to 1.0) of the range of 
 * @param brightness Brightness setting for RGB LEDs (0-255)
 * data array vals to mask off the bottom of the data. If a value follows into
 * that lower portion, it will be represented as LOW on the color scale. Also
 * serves to fit the whole color scale to the upper fraction of data
 * that IS represented.
 */      
void LED_colorscale_set32(float* vals, 
                          CRGB* leds,
                          CRGBPalette32* palette_ptr,
                          int32_t num_leds,
                          uint32_t num_palette,
                          uint8_t data_LED_scale,
                          float mask_low_fraction,
                          uint8_t brightness);
  
#endif  // WS2812B_COLORSCALE_H_