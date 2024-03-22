/**
 * @file WS2812b_colorscale.cpp
 * @brief Functions to drive a WS2812b RGB LED array with HEX color values mapped
 * to an input array of caller-input doubles. Wrutten for Arduino Uno R4 Wifi.
 * Each RGB LED "i" is set to the relative strength of the input array["i"].
 * @copyright Chase Vara, 2024
 **/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <FastLED.h>
#include "WS2812b_colorscale.h"

void LED_colorscale_set32(double* vals, 
                           CRGB* leds,
                           CRGBPalette32* palette_ptr,
                           int32_t num_leds,
                           uint32_t max_palette_index,
                           uint8_t data_LED_scale,
                           double mask_low_fraction,
                           uint8_t brightness) {

  // get max value in data array
  double max = 0.0;
  for (uint32_t i = 0; i < num_leds; i++) {
    double val = vals[i];
    max = (val > max) ? val : max; 
  }

  // value at which a data array item falls below threshold for representation
  // and is thus represnted as LOW color value
  double val_cuttoff = mask_low_fraction * max;
  double max_norm = max - val_cuttoff;

  // map data and set each corresponding LED
  double val;  // input val from array
  double norm_val;  // val after rescaling for threshold
  uint8_t palette_index;
  for (int i = 0; i < num_leds / data_LED_scale; i++) {
    val = vals[i];
    norm_val = val - val_cuttoff;
    // value doesn't surpass threshold, set to low
    if (norm_val < 0) {
      for (int j = 0; j < data_LED_scale; j++) {
        leds[(i * data_LED_scale) + j] = ColorFromPalette(*palette_ptr , 0, brightness);
      }
    // else, map value to colorscale index
    } else {
      palette_index = (uint8_t) ((norm_val / max_norm) * (max_palette_index + 1));

      // clamp color index to maximum index allowed by size of palette
      if (palette_index > max_palette_index) {
        palette_index = max_palette_index;
      }
      // set LED
      for (int j = 0; j < data_LED_scale; j++) {
        leds[(i * data_LED_scale) + j] = ColorFromPalette(*palette_ptr , palette_index, brightness);
      }
    }
  }
  // update LEDs to new settings
  FastLED.show();
}

