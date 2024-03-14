/**
 * @file AudioSampler.h
 * @copyright Chase Vara, 2024
 **/
#ifndef AUDIOSAMPLER_H_
#define AUDIOSAMPLER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

/**
 * @class AudioSampler
 * @brief C++ class to handle audio sampling in an Arduino environment
 * @details Must be configured target MCU. For example, to the RA4M1 MCU
 * on the Arduino Uno Rev 4, sketch must have a "#define __RA4M1__ NULL"
 * statement to configure MCU timers.
*/
class AudioSampler {
 public:
  /**
   * @fn AudioSampler constructor
   * @memberof AudioSampler
   * @brief Initializes AudioSampler object for sampling analog audio data
   * at given desired sample rate and ADC resolution
   * @param f_CPU System clock rate
   * @param f_sample Desired sample rate
   * @param ADC_resolution Bit resolution of ADC associated with pin
   * @param arduino_analog_pin Arduino analog input pin to sample (e.g. "A0");
  */ 
  AudioSampler(uint64_t f_CPU, 
               uint64_t f_sample, 
               uint8_t ADC_resolution, 
               uint32_t arduino_analog_pin);
  ~AudioSampler();

  /**
   * @fn sample
   * @memberof AudioSampler
   * @brief Performs sample
   * @param sampled_data array of sampled date 
   * @param sample_length number of samples to take
   * @details Caller must initialize passed output parameter variables; 
   * sample_length must equal size of sampled_data array.
  */
  void sample(double* sampled_data, 
              uint16_t sample_length);

  /**
   * @fn changeADCres
   * @memberof AudioSampler
   * @brief Changes ADC bit resolution
   * @param ADC_resolution new ADC resolution to change to
  */
  void changeADCres(uint8_t ADC_resolution);

  /**
     * @fn setBaseLine
     * @memberof AudioSampler
     * @brief Changes value threshold for output sample zero value
     * @param ADC_resolution new sample output threshold
    */
  void setBaseLine(uint16_t digital_val_baseline);

 private:
  uint64_t sys_clk_freq_;
  uint64_t sample_rate_;
  uint64_t sample_length_;
  uint8_t ADC_bit_resolution_;
  
  // Arduino analog input pin to sample
  uint32_t analog_in_pin_;

  // threshold value to normalize sample readings at
  uint16_t digital_val_baseline_;

  //  MCU-specific timer timer object
  class _SampleTimer {
   public:
    _SampleTimer(uint64_t period_ticks);
    ~_SampleTimer();
    void config(uint64_t period_ticks);  // does not use prescaler
    void start();
    void stop();
    bool check();  // returns true if counter-timer reached top
    void reset();
    #ifdef __RA4M1__
    R_GPT0_Type* timer_ptr_;
    void config_RA4M1_GPT0(uint64_t period_ticks);
    #endif
  };

  // timer
  _SampleTimer timer_;
};
#endif  // AUDIOSAMPLER_H_