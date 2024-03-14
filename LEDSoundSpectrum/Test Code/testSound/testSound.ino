// Test code for KY-030 big sound sensor
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
// #include "ArduinoRS485.h"
// #include "IRQManager.h"
// #include <ArduinoRS485.h>



#define SOUND_PIN A0
#define ANALOG_READ_RESOLUTION 14

#define F_CPU 48000000

// even division of RA4M1 clock rate (48MHz) 
// and close to 44100 CD-quality audio sampling rate
#define F_SAMPLE 48000  
#define NUM_SAMPLES 256 

double real[NUM_SAMPLES];
int base = 0;

void setup() {
  Serial.begin(115200);

  // Set analog read resolution
  analogReadResolution(ANALOG_READ_RESOLUTION);

  // Set up a sampling timer
  R_MSTP->MSTPCRD &= ~(1 << 5); // cancel GPT module stop state (default stopped after reset)
  R_GPT0_Type* timer;
  timer = R_GPT1;                  // general purpose PWM timer 0
  timer->GTCR &= ~(0b000 << 16);   // Saw-wave PWM mode
  timer->GTUDDTYC |= 0b01;         // count-up mode
  timer->GTCR &= ~(0b000 << 24);    // prescaler 1
  timer->GTPR = (F_CPU / F_SAMPLE) - 1; // cycle top
  timer->GTCNT = 0;               // start value

  // reset overflow flag
  timer->GTST &= ~(1 << 6);

  // start timer
  timer->GTCR |= 1;

  // set baseline analog reading
  base = analogRead(SOUND_PIN);
  delay(2000);

  int i = 0;
  while (i < NUM_SAMPLES) {
    // sample timer reached overflow
    if (timer->GTST & (1 << 6)) {
      // get ADC data from A0
      real[i] = (double) analogRead(SOUND_PIN) - base;

      // reset timer overflow flag
      timer->GTST &= ~(1 << 6);
      // increment to next ieration
      i++;
    }
  }

  for (i = 0; i < NUM_SAMPLES; i++) {
    Serial.print("Sound data #"); Serial.print(i); Serial.print(":  "); 
    Serial.println(real[i]);
  }
  base = real[NUM_SAMPLES - 1];

}

void loop() {
  // Serial.print("Sound data :"); Serial.println(analogRead(SOUND_PIN) - base);
}
