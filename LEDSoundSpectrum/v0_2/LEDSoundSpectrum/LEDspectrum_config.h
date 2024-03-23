/**
 * @file LEDspectrum_config.h
 * @brief configuration settings for target Arduino board/MCU for 
 * LedSoundSpectrum project
 * @copyright Chase Vara, 2024
 **/
#include "color_scales.h"  // for array of heatscale hex colors

//// Conditional Compilation for configuration /////////////////////////////////

// compile for RA4M1 MCU (Arduino Uno R4)
#define __RA4M1__ (1)

// compile to include a delay between computation/LED refresh cycles
#define __USE_DELAY__ (1)

//// debug: print RTOS task memory highwater marks to serial
#define __SERIAL_OUT__  (0)  // enables serial output to monitor
#define __DEBUG_MEM_SUPER__ (0) // check marks for tasks running continuously
#define __DEBUG_MEM_EVENT__ (0) // check marks for for event-driven tasks
#define __DEBUG_DELAYS__ (0)

// monitor task data operations
#define __DEBUG_AUDIO__ (0)  // print out raw ADC readings for audio samples
#define __DEBUG_FFT__ (0)  // print out FFT results
#define __DEBUG_LED__ (0)  // print out color scale indexes mapped to FFT data

// Length of FFT Real data (Real half of FFT data)
// #define __FFT_N_REAL_32__ (1) 
#define __FFT_N_REAL_128__ (1)

// 256 NOT VERFIED TO WORK WITHIN RA4M1 RAM CONSTRAINTS
//#define __FFT_N_REAL_256__ (1)

//// User interaction settings /////////////////////////////////////////////////

// period to check user setting for audio sensing threshold
#define F_SAMPLE 44100 
#define MIN_TICKS_DELAY           20
#define TICKS_FOR_SERIAL          20
#define PERIOD_MILLIS_USER_INPUT  100
#define PERIOD_MILLIS_LED         25
#define PERIOD_MILLIS_AUDIO       50
#define SCALE_FFT_FOR_PARSEVAL    (1)
#define DEBOUNCE_MILLIS             200
#define NUM_LEDS_FFT                128
#define NUM_LEDS_USER               3
#define DATA_LED_SCALE              1  // the number of LEDs that represent 
                                       // one FFT real data point
#define PALETTE_TYPE                CRGBPalette32 // FastLED library palette
#define MAX_FAST_LED_INDEX          255  // for use with FastLED::ColorToPalette,
                                         // which uses normalized indices [0-255]

//// Target board settings /////////////////////////////////////////////////////

// Analog input pin for sampling audio
#define PIN_AUDIO_IN A5

// Analog input fot FFT moving average weight ratio
#define PIN_MA_RATIO A0

// Analog input pin for FFT magnitude threshold setting
#define PIN_THRESHOLD_IN A2

// Analog input pin for LED brightness setting
#define PIN_BRIGHTNESS_IN A4

// User color scale alternation button input
#define PIN_COLOR_SCALE 7

// LED (FFT Display) array control output pin
#define PIN_LED_FFT_CONT 4

// LED (User input settings) array control output pin
#define PIN_LED_USER_CONT 8

////// Target MCU settings /////////////////////////////////////////////////////

#ifdef __RA4M1__
// System clock rate 
#define F_CPU 48000000

// ADC resolution for audio ADC input
#define ANALOG_READ_RESOLUTION 14

// max ADC digital value at ANALOG_READ_RESOLUTION
#define MAX_ADC_VAL ((float) 0x3FFF)  // 0x3FFF is max for 14-bit ADC resolution
#endif  // __RA4M1__

//// Settings for 256-length FFT data (128 real data values) /////////////////////
#ifdef __FFT_N_REAL_128__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           128
#define STACK_DEPTH_SUPERVISOR      128
#define STACK_DEPTH_AUDIO           384
#define STACK_DEPTH_LED             512

// Task priorities
#define PRIORITY_SETUP              1
#define PRIORITY_SUPERVISOR         0
#define PRIORITY_AUDIO              0
#define PRIORITY_LED                0

// Audio sample size
#define NUM_SAMPLES 256
#define NUM_SAMPLES_HALF 128

#endif  // __FFT_N_REAL_128__
