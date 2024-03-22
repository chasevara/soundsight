/**
 * @file config.h
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
#define __DEBUG_MEM_STAT__ (0) // check marks for tasks running continuously
#define __DEBUG_MEM_EVENT__ (0) // check marks for for event-driven tasks

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
// #define USER_SET_THRESH_MILLIS      20
#define DEFAULT_ROUTINE_TASK_DELAY  50
#define SCALE_FFT_FOR_PARSEVAL (1)
#define DEFAULT_DELAY_MILLIS        10
#define DEBOUNCE_MILLIS             200
#define NUM_LEDS                    128
#define DATA_LED_SCALE              1  // the number of LEDs that represent 
                                       // one FFT real data point
#define PALETTE_TYPE                CRGBPalette32 // FastLED library palette
#define MAX_FAST_LED_INDEX          255  // for use with FastLED::ColorToPalette,
                                         // which uses normalized indices [0-255]

//// Target board settings /////////////////////////////////////////////////////

// Analog input pin for sampling audio
#define PIN_AUDIO_IN A5

// Analog input pin for sampling rate setting
#define PIN_SAMPLE_RATE A2

// Analog input fot FFT moving average weight ratio
#define PIN_MA_RATIO A4

// Analog input pin for FFT magnitude threshold setting
#define PIN_THRESHOLD_IN A3

// Analog input pin for LED brightness setting
#define PIN_BRIGHTNESS_IN A1

// User color scale alternation button input
#define PIN_COLOR_SCALE 8

// LED array control output pin
#define PIN_LED_CONT 7
////// Target MCU settings /////////////////////////////////////////////////////

#ifdef __RA4M1__
// System clock rate 
#define F_CPU 48000000

// ADC resolution for audio ADC input
#define ANALOG_READ_RESOLUTION 14

// max ADC digital value at ANALOG_READ_RESOLUTION
#define MAX_ADC_VAL (double) 0x3FFF  // 0x3FFF is max for 14-bit ADC resolution
#endif  // __RA4M1__

//// Settings for 64-length FFT data (32 real data values) /////////////////////
#ifdef __FFT_N_REAL_32__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           1 << 9
#define STACK_DEPTH_DELAY           1 << 7
#define STACK_DEPTH_AUDIO           1 << 8
#define STACK_DEPTH_FFT             1 << 8
#define STACK_DEPTH_LED             1 << 7

// Task priorities
#define PRIORITY_SETUP              6
#define PRIORITY_AUDIO              5
#define PRIORITY_FFT                4
#define PRIORITY_LED                3
#define PRIORITY_DELAY              2

// Audio sample rate
#define F_SAMPLE_MAX 44100 

// Audio sample size
#define NUM_SAMPLES 64
#define NUM_SAMPLES_HALF 32

#endif  // __FFT_N_REAL_32__

//// Settings for 256-length FFT data (128 real data values) /////////////////////
#ifdef __FFT_N_REAL_128__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           128
#define STACK_DEPTH_DELAY           50
#define STACK_DEPTH_AUDIO           100
#define STACK_DEPTH_FFT             400
#define STACK_DEPTH_LED             110
#define STACK_DEPTH_COLORS          156

// Task priorities
#define PRIORITY_SETUP              6
#define PRIORITY_AUDIO              5
#define PRIORITY_FFT                4
#define PRIORITY_LED                3
#define PRIORITY_DELAY              2
#define PRIORITY_COLORS             0

// Audio sample rate
#define F_SAMPLE_MAX 44100 

// Audio sample size
#define NUM_SAMPLES 256
#define NUM_SAMPLES_HALF 128

#endif  // __FFT_N_REAL_64__

//// Settings for 256-length real FFT data /////////////////////////////////////
#ifdef __FFT_N_REAL_256__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           1 << 8
#define STACK_DEPTH_DELAY           1 << 7
#define STACK_DEPTH_SAMPLERATE      1 << 7
#define STACK_DEPTH_AUDIO           1 << 9
#define STACK_DEPTH_FFT             1 << 9
#define STACK_DEPTH_SET_THRESHOLD   1 << 7
#define STACK_DEPTH_LED             1 << 

// Task priorities
#define PRIORITY_SETUP              6
#define PRIORITY_SAMPLERATE         1
#define PRIORITY_DELAY              3
#define PRIORITY_AUDIO              5
#define PRIORITY_FFT                4
#define PRIORITY_SET_THRESHOLD      0
#define PRIORITY_LED                5

// Audio sample rate
#define F_SAMPLE 78400 

// Audio sample size
#define NUM_SAMPLES 512 
#define NUM_SAMPLES_HALF 256

#endif  // __FFT_N_REAL_256__
