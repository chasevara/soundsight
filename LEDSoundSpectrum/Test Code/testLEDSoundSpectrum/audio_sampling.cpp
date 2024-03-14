/**
 * @file audio_sampling.cpp
 * @brief functions to perform audio sampling, timed by an RA4M1 MCU General
 * PWM Timer
 * @details functions require RA4M1 timer peripherals to be enabled:
 * @copyright Chase Vara, 2024
 **/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "audio_sampling.h"

void sample(double* sample_data,
            double f_clock,
            double f_sample,
            uint16_t sample_N,
            uint32_t analog_in_pin) {

  // Tetup timer
  R_GPT0_Type* timer_ptr = R_GPT1; // general PWM timer 1
  sampling_config(f_clock, f_sample, timer_ptr);

  // start timer
  timer_ptr->GTCR |= 1;

  // Take each sample when the timer completes its period
  for (int i = 0; i < sample_N; i++) {
    // sample timer reached top, so take sample
    if (timer_ptr->GTST & (1 << 6)) {

      // get ADC data from A0
      sample_data[i] = (double) analogRead(analog_in_pin);

      // reset timer overflow flag
      timer_ptr->GTST &= ~(1 << 6);
    } else {
      // Sample not taken, stay on this sample index
      i--;
    }
  }
  // Stop and disable timer
  sampling_cleanup(timer_ptr);
}

static void sampling_config(double f_clock, 
                            double f_sample, 
                            R_GPT0_Type* timer_ptr) {
  // Enable timers
  R_MSTP->MSTPCRD &= ~(1 << 5);

  // Set up a sampling timer
  timer_ptr->GTCR &= ~(0b000 << 16);         // Saw-wave PWM mode
  timer_ptr->GTUDDTYC |= 0b01;               // count-up mode
  timer_ptr->GTCR &= ~(0b000 << 24);         // prescaler 1
  timer_ptr->GTPR = (f_clock / f_sample) - 1;  // cycle top
  timer_ptr->GTCNT = 0;                      // start value

  // Reset overflow flag
  timer_ptr->GTST &= ~(1 << 6);
}

static void sampling_cleanup(R_GPT0_Type* timer_ptr) {
  // stop timer
  timer_ptr->GTCR &= ~1;

  // disable timers
  R_MSTP->MSTPCRD |= (1 << 5);
}
