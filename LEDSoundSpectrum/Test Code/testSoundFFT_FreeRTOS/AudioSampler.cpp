/**
 * @file AudioSampler.cpp
 * @brief Implementation of AudioSampler class declared in AudioSampler.h
 * @copyright Chase Vara, 2024
 **/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "AudioSampler.h"
#include <Arduino_FreeRTOS.h>

AudioSampler::AudioSampler(uint64_t f_CPU, 
                           uint64_t f_sample, 
                           uint8_t ADC_resolution, 
                           uint32_t arduino_analog_pin)
  :
  sys_clk_freq_(f_CPU),
  sample_rate_(f_sample),
  ADC_bit_resolution_(ADC_resolution),
  analog_in_pin_(arduino_analog_pin),
  digital_val_baseline_(0),
  timer_(static_cast<uint64_t>(static_cast<double>(f_CPU) / f_sample)) {
  // Serial.println("Audio Sampler created");
}

AudioSampler::~AudioSampler() {
  timer_.stop();
}

void AudioSampler::sample(double* sampled_data, 
                          uint16_t sample_length) {
  timer_.start();
  for (uint16_t i = 0; i < sample_length;) {
    // sample timer completed cycle --> take sample
    if (timer_.check()) {
      // set ADC bit resolution
      analogReadResolution(ADC_bit_resolution_);

      // get ADC data from analog in pin
      int16_t sample = analogRead(analog_in_pin_);
      if (sample <= digital_val_baseline_) {
        sample = 0;
      } else {
        sample -= digital_val_baseline_;
      }
      sampled_data[i] = (double) sample;

      // reset timer overflow flag
      timer_.reset();

      // increment to next ieration
      i++;
    }
    // else stay on this sample until timer complete
    i--;
  }
  timer_.stop();
}

void AudioSampler::changeADCres(uint8_t ADC_resolution) {
  ADC_bit_resolution_ = ADC_resolution;
  analogReadResolution(ADC_resolution);
}

void AudioSampler::setBaseLine(uint16_t digital_val_baseline) {
  digital_val_baseline_ = digital_val_baseline;
}

AudioSampler::_SampleTimer::_SampleTimer(uint64_t period_ticks) {
  config(period_ticks);
}

AudioSampler::_SampleTimer::~_SampleTimer() {
  stop();
}

void AudioSampler::_SampleTimer::config(uint64_t period_ticks) {
  #ifdef __RA4M1__
  config_RA4M1_GPT0(period_ticks);
  #endif
}

void AudioSampler::_SampleTimer::start() {
  #ifdef __RA4M1__
  timer_ptr_->GTCR |= 1;
  #endif
}

void AudioSampler::_SampleTimer::stop() {
  #ifdef __RA4M1__
  timer_ptr_->GTCR &= ~1;
  #endif
}

bool AudioSampler::_SampleTimer::check() {
  #ifdef __RA4M1__
  return timer_->GTST & (1 << 6);
  #endif
}

void AudioSampler::_SampleTimer::reset() {
  #ifdef __RA4M1__
  timer_ptr_->GTST &= ~(1 << 6);
  #endif
}

#ifdef __RA4M1__
// Refer to RA4M1 hardware user guide for more information re: configuration
void AudioSampler::_SampleTimer::config_RA4M1_GPT0(uint64_t period_ticks) {
  // cancel GPT module stop state (default stopped after UNO R4 reset)
  R_MSTP->MSTPCRD &= ~(1 << 5);

  timer_ptr_ = R_GPT0;                        // general purpose PWM timer 0
  timer_ptr_->GTCR &= ~(0b000 << 16);         // Saw-wave PWM mode
  timer_ptr_->GTUDDTYC |= 0b01;               // count-up mode
  timer_ptr_->GTCR &= ~(0b000 << 24);         // prescaler 1
  timer_ptr_->GTPR = 
    (sys_clk_freq_ / sample_rate_) - 1;  // cycle top
  timer_ptr_->GTCNT = 0;                      // start value

  // reset overflow flag
  timer_ptr_->GTST &= ~(1 << 6);
}
#endif