/**
 * @file WS2812b_colorscale.cpp
 * @brief Functions to drive a WS2812b RGB LED array with HEX color values mapped
 * to an input array of caller-input floats. Wrutten for Arduino Uno R4 Wifi.
 * Each RGB LED "i" is set to the relative strength of the input array["i"].
 * @copyright Chase Vara, 2024
 **/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <FastLED.h>
#include "WS2812b_colorscale.h"
#include "config.h"

void LED_colorscale_set32(float* vals, 
                           CRGB* leds,
                           CRGBPalette32* palette_ptr,
                           int32_t num_leds,
                           uint32_t max_palette_index,
                           uint8_t data_LED_scale,
                           float mask_low_fraction,
                           uint8_t brightness) {

  if (__DEBUG_LED__) {
    Serial.println("--------- LED INPUT BEGIN ---------\n");
    for (int i = 0; i < NUM_SAMPLES_HALF; i++) {
      Serial.print("\n  LED Input Data #"); Serial.print(i);
      Serial.print("  ----  "); Serial.println(vals[i]);
    }
    Serial.println("---------- LED INPUT END ----------\n\n");
  }

  // get max value in data array
  float max = 0.0;
  for (uint32_t i = 1; i < num_leds; i++) {  // don;t count DC component
    float val = vals[i];
    max = (val > max) ? val : max; 
  }

  // value at which a data array item falls below threshold for representation
  // and is thus represnted as LOW color value
  float val_cuttoff = mask_low_fraction * max;
  float max_norm = max - val_cuttoff;

  // map data and set each corresponding LED, shifting data left, so that DC component
  // is excluded
  float val;  // input val from array
  float norm_val;  // val after rescaling for threshold
  uint8_t palette_index;

  if (__DEBUG_LED__) {
    Serial.print("FFT max val: "); Serial.println(max);
    Serial.print("FFT threshold cuttoff val: "); Serial.println(val_cuttoff);
    Serial.print("Normalized max val: "); Serial.println(max_norm);
    Serial.println("--------- LED NORMALIZED VALS BEGIN ---------\n");
  }

  for (int i = 1; i < num_leds; i++) {
    val = vals[i];
    norm_val = val - val_cuttoff;

    // value doesn't surpass threshold, set to low
    if (norm_val < 0) {

      if (__DEBUG_LED__) {
        Serial.print("  LED Setting #"); Serial.print(i - 1);
        Serial.print("  ----  "); Serial.println(0);
      }
      
      leds[i - 1] // led represents data index shifted by 1
        = ColorFromPalette(*palette_ptr , 0, brightness);
    // else, map value to colorscale index
    } else {

      if (__DEBUG_LED__) {
        Serial.print("  LED Setting #"); Serial.print(i - 1);
        Serial.print("  ----  "); Serial.print(norm_val);
      }

      palette_index 
        = (uint8_t) ((norm_val / max_norm) * (max_palette_index));

      // clamp color index to maximum index allowed by size of palette
      if (palette_index > max_palette_index) {
        palette_index = max_palette_index;
      }

      if (__DEBUG_LED__) {
        Serial.print("  ----  palette index: "); Serial.println(palette_index);
      }

      // set LED
      leds[i - 1] // led represents data index shifted by 1
        = ColorFromPalette(*palette_ptr , palette_index, brightness);
    }
    // set last LED to its neighbor's setting since we shifted data left
    leds[num_leds - 1] = leds[num_leds - 2]; 
  }

  if (__DEBUG_LED__) {
    Serial.println("---------- LED NORMALIZED VALS END ----------\n\n");
  }

  // update LEDs to new settings
  FastLED.show();
}

