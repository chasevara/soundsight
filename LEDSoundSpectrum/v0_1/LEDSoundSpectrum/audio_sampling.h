/**
 * @file audio_sampling.h
 * @brief functions to perform audio sampling, timed by an RA4M1 MCU General
 * PWM Timer
 * @details functions require RA4M1 timer peripherals to be enabled:
 * "R_MSTP->MSTPCRD &= ~(1 << 5); // cancel GPT module stop state"
 * @copyright Chase Vara, 2024
 **/
#ifndef _AUDIO_SAMPLNG_H_
#define _AUDIO_SAMPLNG_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

/**
 * @fn sample
 * @brief samples the parameter analog input pin at the configured sample rate
 * @param sample_data Output parameter for sampled data array
 * @param f_clock System clock frequency
 * @param f_sample Desired sample rate
 * @param sample_N Number of samples to take 
 * (should be length of sample_data)
 * @param analog_in_pin register address of analog input pin to be sampled
 * @return array sampled data (converted to digital value) through sample_data
 * output parameter.
 * @details caller must allocate array output return parameter prior to call.
*/
void sample(float* sample_data,
            float f_clock,
            float f_sample,
            uint16_t sample_N,
            uint32_t analog_in_pin);

/**
 * @fn sampling_config
 * @brief Helper function 'sample' Configures RA4M1 GPT1 as a one-shot saw-tooth 
 * wave up-counter to drive periodic sampling
 * @param f_clock System clock frequency
 * @param f_sample Desired sample rate for timer 
 * @param timer_ptr Pointer to RA4M1 GPT timer register struct 
 * @details f_sample should be twice the the maximum freuency the caller wants
 * to sample.
*/
static void sampling_config(float f_clock, 
                            float f_sample, 
                            R_GPT0_Type* timer_ptr);

/**
 * @fn sampling_cleanup
 * @param timer_ptr Pointer to RA4M1 GPT timer register struct 
 * @brief Stops RA4M1 GPT1 timer and disables timers (conserve power)
*/
static void  sampling_cleanup(R_GPT0_Type* timer_ptr);

#endif  // _AUDIO_SAMPLNG_H_