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
#include "WS2812b_colorscale.h"
#include "LEDspectrum_config.h"

#define SET_LEDS_OFF CRGB(0, 0, 0)

/**
 * @fn LED_colorscale_set
 * @brief Function that maps input value array to color scale and updates the
 * WS2812b settings to represent the resultant colorscale (32 color values)
 * @param vals_avg Array of value weighted moving averages mapped directly to an
 * the array of LED settings
 * @param vals_update New values to update vals_avg with, 
 * given weight in vals_avg based on weight_moving_avg
 * @param leds Array of current LED array settings
 * @param palette_ptr Pointer to FastLED color palette object
 * @param num_leds The number of serial 5050SMD LED units to drive in the array
 * @param max_palette_index max index of CRGGM color palette for color scale
 * @param threshold_ratio The normal ratio (0.0 to 1.0) of the range of
 * data array vals to mask off the bottom of the data. If a value follows into
 * that lower portion, it will be represented as LOW on the color scale. Also
 * serves to fit the whole color scale to the upper fraction of data
 * that IS represented.
 * @param brightness Brightness setting for RGB LEDs (0-255)
 * @param weight_moving_avg Weight (0.0-1.0) of contribution of val_update[i] to 
 * updated value of val_avg[i]; If the val_update[i] is larger than val_avg[i], weight
 * will default to 1.0;
 */      
void LED_colorscale_set(float* vals_avg,
                        float* vals_update,
                        CRGB* leds,
                        CRGBPalette32* palette_ptr,
                        int32_t num_leds,
                        uint32_t max_palette_index,
                        float threshold_ratio,
                        uint8_t brightness,
                        float weight_moving_avg);

#endif  // WS2812B_COLORSCALE_H_