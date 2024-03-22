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

void LED_colorscale_set(float* vals_avg, 
                        float* vals_update, 
                        CRGB* leds,
                        CRGBPalette32* palette_ptr,
                        int32_t num_leds,
                        uint32_t max_palette_index,
                        float threshold_ratio,
                        uint8_t brightness,
                        float weight_moving_avg) {
  float max_avg;
  float val_avg;
  float val_update;
  float val_avg_norm;
  float val_cuttoff;
  float max_norm;

  if (__DEBUG_LED__) {
    Serial.println("--------- LED INPUT BEGIN ---------\n");
    for (int i = 0; i < num_leds; i++) {
      Serial.print("  LED Input Data #"); Serial.print(i);
      Serial.print("  ----  "); Serial.println(vals_update[i]);
    }
    Serial.println("---------- LED INPUT END ----------\n\n");
  }

  // Update value weighted moving averages (starting at index 1; don't care 
  // about DC component at [0])
  for (int i = 1; i < num_leds; i++) { 
    val_update = vals_update[i];
    // cast operands to ints to avoid hanging on 
    // floating point addition error
    vals_avg[i] = vals_avg[i] + (weight_moving_avg * (val_update - vals_avg[i]));
    // update erroneous negative value (caused by floating point ops) to 0
    vals_avg[i] = (vals_avg[i] < 0) ? 0 : vals_avg[i];
  }

  // get max value in weighted moving averages array
  max_avg = 0.0;
  for (uint32_t i = 1; i < num_leds; i++) {  // don't count DC component
    val_avg = vals_avg[i];
    max_avg = (val_avg > max_avg) ? val_avg : max_avg; 
  }

  // get value at which a data array item falls below threshold for 
  // representation and is thus represented as LOW color value 
  val_cuttoff = threshold_ratio * max_avg;
  max_norm = max_avg - val_cuttoff;

  if (__DEBUG_LED__) {
    Serial.println("--------- LED OUTPUT BEGIN ---------");
    Serial.print("Max weighted avg val: "); Serial.println(max_avg);
    Serial.print("Threshold ratio setting: "); Serial.println(threshold_ratio);
    Serial.print("MA weight setting: "); Serial.println(weight_moving_avg);
    Serial.print("\n\n");
  }

  // Update LED settings, normalizing for threshold ratio input;
  // shift <data : LED index> correlation left by 1 since we don't represent DC
  // component
  uint8_t palette_index;
  for (int i = 1; i < num_leds; i++) {
    val_avg = vals_avg[i];
    val_avg_norm = val_avg - val_cuttoff;

    // If new value of data point doesn't surpass threshold, set to 0
    val_avg_norm = (val_avg_norm < 0) ? 0 : val_avg_norm;

    // map corresponding LED color setting to weigted average
    palette_index 
      = (uint8_t) ((val_avg_norm / max_norm) * (max_palette_index));

    // clamp color index to maximum index allowed by size of palette
    palette_index 
      = (palette_index > max_palette_index) ? max_palette_index : palette_index;
      
    // update correlated LED setting 
    leds[i - 1] = ColorFromPalette(*palette_ptr , palette_index, brightness);

    if (__DEBUG_LED__) {
      Serial.print("\n  LED Color Index #"); Serial.print(i);
      Serial.print("  ----  : "); Serial.print(palette_index);
    }
  }
  // set last LED to its neighbor's setting since we shifted 
  // <data : LED index> correlation 
  leds[num_leds - 1] = leds[num_leds - 2]; 

  if (__DEBUG_LED__) {
    Serial.println("---------- LED OUTPUT END ----------\n\n");
  }

  // update LEDs to new settings
  FastLED.show();
}


