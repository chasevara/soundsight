// Test code for controlling WS2812b RGB strip (144 lights)
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    144
#define LED_DELAY_MILLIS 100
#define LED_DELAY_MICROS 128

#define SET_RED CRGB(255, 0, 0)
#define SET_GREEN CRGB(0, 255, 0)
#define SET_BLUE CRGB(0, 0, 255)
#define SET_OFF CRGB(0, 0, 0)

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(19200);

  // initialize LED controller objects
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  // set strip to off
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = SET_OFF;
    FastLED.show();
  }
}

void loop() {
  // for(int i = 0; i < NUM_LEDS; i++) {
  //   Serial.println("Testing next light");
  //   for(int j = 0; j <= 200; j = j + 10) {
  //     leds[i] = CRGB(j, 0, 0);
  //     FastLED.show();
  //     delayMicroseconds(LED_DELAY_MICROS);
  //   }
  //   for(int k = 200; k >= 0; k = k - 10) {
  //     leds[i] = CRGB(k, 0, 0);
  //     FastLED.show();
  //     delayMicroseconds(LED_DELAY_MICROS);
  //   }
  // }
  int a = 0;
  int n = 256;
  srand(millis());
  int red_nums[NUM_LEDS];
  int green_nums[NUM_LEDS];
  int blue_nums[NUM_LEDS];

  for(int i = 0; i < NUM_LEDS; i++) {
    red_nums[i] = a + rand() % n;
    green_nums[i] = a + rand() % n;
    blue_nums[i] = a + rand() % n;
  }

  int starttime = millis();
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(red_nums[i], green_nums[i], blue_nums[i]);
  }
  FastLED.show();
  int endtime = millis();
  Serial.print("Time to set LED strip: "); Serial.println(endtime - starttime);
  delay(LED_DELAY_MILLIS);
}
