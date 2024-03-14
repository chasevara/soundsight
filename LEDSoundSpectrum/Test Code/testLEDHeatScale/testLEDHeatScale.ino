// Test code for driving WS2812b RGB LED strip with 
// magnitude --> RGB heatscale color value settings 
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <FastLED.h>
#include "heat_scale.h"

#define LED_PIN     7
#define ANALOG_PIN A2
#define ADC_BIT_RES 14
#define NUM_LEDS    144
#define LED_DELAY_MILLIS 30

#define SET_RED CRGB(255, 0, 0)
#define SET_GREEN CRGB(0, 255, 0)
#define SET_BLUE CRGB(0, 0, 255)
#define SET_OFF CRGB(0, 0, 0)

#define SCALE_LENGTH 256

CRGB leds[NUM_LEDS];
uint16_t bit_resolution = ADC_BIT_RES;
double kmax_magnitude = (double) (1 << bit_resolution) - 1;

const uint32_t heatScaleHex[SCALE_LENGTH] = HEAT_SCALE_HEX256;
CRGBPalette256 palette;

void setup() {
  Serial.begin(115200);

  // initialize FastLED color palette
  for (int i = 0; i < SCALE_LENGTH; i++) {
    palette[i] = CRGB(heatScaleHex[i]);
  }

  // initialize LED controller objects
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  // set strip to off
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = SET_OFF;
  }
  FastLED.show();

  // Set analog read resolution
  analogReadResolution(ADC_BIT_RES);
}

void loop() {

  //int starttime = millis();
  double magnitude = analogRead(ANALOG_PIN);
  //Serial.print("ADC Reading: "); Serial.print(magnitude); Serial.print("  /  Max Value Possible: "); Serial.println(kmax_magnitude);

  // get scale index of magnitude
  //Serial.print("Heat scale index: "); Serial.println(index);
  // set LEDS to colors for palette index
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t index = (uint32_t) (SCALE_LENGTH * magnitude / kmax_magnitude);
    leds[i] = ColorFromPalette(palette, index);
  }
  FastLED.show();

  //int endtime = millis();
  //Serial.print("Time to set LEDs: "); Serial.println(endtime - starttime);
  delay(LED_DELAY_MILLIS);
}
