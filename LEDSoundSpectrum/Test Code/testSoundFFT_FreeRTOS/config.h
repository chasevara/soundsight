/**
 * @file config.h
 * @brief configuration settings for target Arduino board/MCU for 
 * Audiograph project
 * @copyright Chase Vara, 2024
 **/

//// Conditional Compilation for configuration /////////////////////////////////

// compile for RA4M1 MCU (Arduino Uno R4)
#define __RA4M1__ (1)

// debug: print RTOS task memory highwater marks to serial
//#define __DEBUG_MEM_STAT__ (1) // check marks for tasks running continuously
#define __DEBUG_MEM_EVENT__ (1) // check marks for for event-driven tasks

// Length of FFT Real data (Real half of FFT data)
#define __FFT_N_REAL_128__ (1) 

// 256 NOT VERFIED TO WORK WITHIN RA4M1 RAM CONSTRAINTS
//#define __FFT_N_REAL_256__ (1)

//// User interaction settings /////////////////////////////////////////////////

// period to check user setting for audio sensing threshold
#define USER_SET_THRESH_MILLIS 100
#define USER_SET_DELAY_MILLIS 100
#define DEFAULT_DELAY_MILLIS 250

//// Target board settings /////////////////////////////////////////////////////

#ifdef __RA4M1__
// System clock rate 
#define F_CPU 48000000

// Analog input pin for sampling audio
#define PIN_AUDIO_IN A0

// Analog input pin for audio baseline threshold setting
#define PIN_THRESHOLD_IN A2

// ADC resolution for audio ADC input
#define ANALOG_READ_RESOLUTION_AUDIO 12
#define ANALOG_READ_RESOLUTION_THRESHOLD 10

#endif  // __RA4M1__

//// Settings for 128-length real FFT data /////////////////////////////////////
#ifdef __FFT_N_REAL_128__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           1 << 7
#define STACK_DEPTH_DELAY           1 << 6
#define STACK_DEPTH_AUDIO           (1 << 8) + (1 << 7)
#define STACK_DEPTH_FFT             1 << 8
#define STACK_DEPTH_SET_THRESHOLD   1 << 6
#define STACK_DEPTH_PrintFFT        1 << 7

// Task priorities
#define PRIORITY_SETUP              6
#define PRIORITY_SET_DELAY          3
#define PRIORITY_DELAY              2
#define PRIORITY_AUDIO              5
#define PRIORITY_FFT                4
#define PRIORITY_SET_THRESHOLD      1
#define PRIORITY_PRINT              0

// Audio sample rate
#define DEFAULT_SAMPLE_RATE 44100 

// Audio sample size
#define NUM_SAMPLES 32

#endif  // __FFT_N_REAL_128__

//// Settings for 256-length real FFT data /////////////////////////////////////
#ifdef __FFT_N_REAL_256__
/* FreeRTOS settings
 */
// Task stack depths
#define STACK_DEPTH_SETUP           1 << 8
#define STACK_DEPTH_DELAY           1 << 7
#define STACK_DEPTH_AUDIO           1 << 8
#define STACK_DEPTH_FFT             1 << 9
#define STACK_DEPTH_SET_THRESHOLD   1 << 8
#define STACK_DEPTH_PrintFFT        1 << 9

// Task priorities
#define PRIORITY_SETUP              6
#define PRIORITY_SET_DELAY          3
#define PRIORITY_DELAY              2
#define PRIORITY_AUDIO              5
#define PRIORITY_FFT                4
#define PRIORITY_SET_THRESHOLD      1
#define PRIORITY_PRINT              0

// Audio sample rate
#define F_SAMPLE 78400 

// Audio sample size
#define NUM_SAMPLES 512 

#endif  // __FFT_N_REAL_256__
