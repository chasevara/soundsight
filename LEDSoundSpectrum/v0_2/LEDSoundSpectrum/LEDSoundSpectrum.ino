/**
 * @file LEDSoundSpectrum.ino
 * @brief Main program to synchronize audio sampling, FFT computation, and
 * LED settings to affect a real-time visual spectrum analysis of ambient 
 * sound. Board, MCU, LED , and debug settings are found in config.h.
 * @details Program is organized as a FreeRTOS scheduler.
 * @copyright Chase Vara, 2024
 **/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <arm_math.h>
#include <FastLED.h>
#include "LEDspectrum_config.h"
#include "color_scales.h"
#include "audio_sampling.h"
#include "WS2812b_colorscale.h"

//// FreeRTOS data queues and taskhandles for task synchronization /////////////

QueueHandle_t Q_audio_data;
QueueHandle_t Q_fft_output;
QueueHandle_t Q_fft_moving_avg;
QueueHandle_t Q_palette;
QueueHandle_t Q_threshold;
QueueHandle_t Q_brightness;
QueueHandle_t Q_moving_avg_weight;
QueueHandle_t Q_palette_sel;

TaskHandle_t T_fft;
TaskHandle_t T_audio;

//// FreeRTOS task routine prototypes //////////////////////////////////////////

/**
 * @fn Task_Setup
 * @brief Task function to setup the MCU/Board for FreeRTOS-scheduled operations
 * @details This task allocates the LED settings queues on the heaps and 
 * launches the supervisor task; deletes itself after performing initilization
 * tasks
*/
void Task_Setup(void *pvParameters);

/**
 * @fn Task_Supervisor
 * @brief Task function to launch audio sampling and LED settings tasks on 
 * and startup, and to enact user inputs for brightness, relative threshold for 
 * frequency display, and visual persistence settings.
 *
 * (USER BRIGHTNESS): uses es an analog input to drive a brightness ratio which which
 * collectively adjusts the RGB color settings to uniformly control brightness
 * 
 * (USER THRESHOLD): Sets the threshold (as a percentrage of the max FFT component 
 * magnitude recived) below which FFT component magnitude data will be excluded.
 * The task also re-scales the valid data above the threshold to properly map to
 * to the full color-scale range. This has the effect of allowing the user to
 * dynamically adjust LEDs to show dominent frequency components of the sampled
 * audio.
 * 
 * (USER PERSISTENCE): Affects the visual peristence of FFT data diaplayed by
 * LEDs by setting a relative weight value used by LED setting tasks to 
 * time-decimate previous FFT data through a weighted moving average. 
 * The weight (0.0-1.0) affects what percentage of the moving average of an FFT
 * datapoint's will be weighted toward the newest FFT data point update.
*/
void Task_Supervisor(void *pvParameters);

/**
 * @fn Task_SetLEDs
 * @brief Maps magnitude data received from Task_FFT to WS2812b RGB LEDs, with
 * with LED indice value corresponding to location on the audio frequency 
 * spectrum for the Task_SampleAudio's set sample rate.
 * @details Color mapping is performed against the color scale defined in
 * config.h.
*/
void Task_SetLEDs(void *pvParameters);

/**
 * @fn Task_SampleAudio
 * @brief Samples audio data from analog input (input pin set in config.h).
 * @details The sample rate is set in config.h; Launches FFT task during 
 * initilialization. Task delays itself accordinf to "delay_audio" milliseconds 
 * setting in config.h;
*/
void Task_SampleAudio(void *pvParameters);

/**
 * @fn Task_FFT
 * @brief Performs an N-point FFT on the the audio sample generated by 
 * Task_SampleAudio (N set in config.h)
 * @details Computes FFT, removes the DC-component from the computed FFT 
 * and converts computed complex data into real magnitudes; Tasks suspends
 * itself upon initilization in order to wait for audio sampling task to 
 * call its operations.
*/
void Task_FFT(void *pvParameters); 

/**
 * @fn debug_mem_to_output
 * @brief Helper function to print calling FreeRTOS task's stack highwater mark
 * (quantified by system memory wordlength) and the current heap stats 
 * (in bytes)
 * @param task_name String representing the name of the task being deubgged
*/
static void debug_mem_to_output(char* task_name);

/**
 * @fn delay_align_to_period
 * @brief Helper function to invoke a delay at the end of a task to align task
 * execution to a desired millisecond period.
 * @param start_ticks FreeRTOS scheduler ticks at which task execution was started
 * @param end_ticks FreeRTOS scheduler ticks at which task execution was ended
 * @param period_millis millisecond period to align task execution to
 * @details Function will only invoke a delay if period_millis ticks minus 
 * (elapsed execution ticks) is greater than or equal to MIN_TICKS_DELAY
*/
static void delay_align_to_period(TickType_t start_ticks, 
                                  TickType_t end_ticks, 
                                  int period_millis);

//// Main program //////////////////////////////////////////////////////////////

void setup() {
  if (__SERIAL_OUT__) {
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect.
    }
  }

  // Launch setup task
  if(xTaskCreate(Task_Setup,
              "Setup Task",
              STACK_DEPTH_SETUP,
              NULL,
              PRIORITY_SETUP,
              NULL) != pdPASS) {
    Serial.println("Failed to create setup task");        
  }
  // Start scheduler
  vTaskStartScheduler() ;
}
void loop() {}


//// Task Definitions //////////////////////////////////////////////////////////

void Task_Setup(void *pvParameters) {
  // Initialize digital I/O pins
  pinMode(PIN_COLOR_SCALE, INPUT);
  pinMode(PIN_LED_FFT_CONT, OUTPUT);
  pinMode(PIN_LED_USER_CONT, OUTPUT);

  #ifdef __RA4M1__
  R_MSTP->MSTPCRD &= ~(1 << 5); // cancel GPT module stop state
  #endif  // __RA4M1__

  // Set analog read resolution
  analogReadResolution(ANALOG_READ_RESOLUTION);

  // Launch supervisor task
  bool created = false;
  created = xTaskCreate(Task_Supervisor,
              "Supervisor Task",
              STACK_DEPTH_SUPERVISOR,
              NULL,
              PRIORITY_SUPERVISOR,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Supervisor Task");
  } 
  // Get the stack high water mark and heap stats for debugging
  if (__DEBUG_MEM_EVENT__) {
    debug_mem_to_output(pcTaskGetName(NULL));
  }
  vTaskDelete(NULL);
}

void Task_Supervisor(void *pvParameters) {
  // variables for measuring execution time and determining task ending delay
  TickType_t start_tick, end_tick, elapsed_ticks;

  // Temp pointer variables for manipulating queue data during steady-state ops
  float* float_data_ptr;
  uint8_t* byte_data_ptr;

  // Setup user setting variables and their queues
  float thresh = (float) analogRead(PIN_THRESHOLD_IN) / MAX_ADC_VAL;
  float* thresh_ptr = &thresh;
  Q_threshold = xQueueCreate(1, sizeof(float*));
  
  float bright = (float) analogRead(PIN_BRIGHTNESS_IN) / MAX_ADC_VAL;
  float* bright_ptr = &bright;
  Q_brightness = xQueueCreate(1, sizeof(float*));

  float weight = (float) analogRead(PIN_MA_RATIO) / MAX_ADC_VAL;
  float* weight_ptr = &weight;
  Q_moving_avg_weight = xQueueCreate(1, sizeof(float*));

  uint8_t palette_sel = 0;
  uint8_t* palette_sel_ptr = &palette_sel;
  Q_palette_sel = xQueueCreate(1, sizeof(uint8_t*));

  // Create FFT output data variable for sharing between FFT and LED tasks
  float* fft_output = (float*) pvPortMalloc(NUM_SAMPLES * sizeof(float));
  for (int i = 0; i < NUM_SAMPLES; i++) {
    fft_output[i] = 0.0;
  }
  Q_fft_output = xQueueCreate(1, sizeof(float*));
  if(xQueueSend(
    Q_fft_output, &fft_output, portMAX_DELAY) != pdPASS) {
    Serial.println("SUPERVISOR: Failed to send Q_fft_output.");
  }

  // Launch audio sampling and LED update tasks
  bool created = false;
  created = xTaskCreate(Task_SampleAudio,
              "Audio Sampler Task",
              STACK_DEPTH_AUDIO,
              NULL,
              PRIORITY_AUDIO,
              &T_audio);
  if (!created) {
    Serial.println("SUPERVISOR: Failed to create Audio Task");
  }
  created =  xTaskCreate(Task_SetLEDs,
              "LED-Setting Task",
              STACK_DEPTH_LED,
              NULL,
              PRIORITY_LED,
              NULL);
  if (!created) {
    Serial.println("SUPERVISOR: Failed to create LED Task");
  }

  // Send created queues to tasks
  if(xQueueSend(
    Q_threshold, &thresh_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("SUPERVISOR: Failed to send Q_threshold.");
  }
  if(xQueueSend(
    Q_brightness, &bright_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("SUPERVISOR: Failed to send Q_brightness.");
  }
  if(xQueueSend(
    Q_moving_avg_weight, &weight_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("SUPERVISOR: Failed to send Q_moving_avg_weight.");
  }
  if(xQueueSend(
    Q_palette_sel, &palette_sel_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("SUPERVISOR: Failed to send Q_palette_sel.");
  }

  // Steady state operations to update user input-affected settings
  for (;;) {
    // Time operations to align ending delay with total desired
    // execution period
    start_tick = xTaskGetTickCount();

    // Update user color scale selection setting if button pressed
    if (digitalRead(PIN_COLOR_SCALE)) {
      xQueueReceive(Q_palette_sel, &byte_data_ptr, portMAX_DELAY);
      *byte_data_ptr = ((*byte_data_ptr) + 1) % NUM_COLOR_SCALES;
      // debounce button with delay
      if(xQueueSend(
        Q_palette_sel, &byte_data_ptr, portMAX_DELAY) != pdPASS) {
        Serial.println("SUPERVISOR: Failed to send Q_palette_sel.");
      }
      vTaskDelay(DEBOUNCE_MILLIS / portTICK_PERIOD_MS);
      continue;
    }

    // Update user brightness setting
    xQueueReceive(Q_brightness, &float_data_ptr, portMAX_DELAY);
    *float_data_ptr = (float) analogRead(PIN_BRIGHTNESS_IN) / MAX_ADC_VAL;
    if(xQueueSend(
      Q_brightness, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("SUPERVISOR: Failed to send Q_brightness.");
    }

    // Update user threshold setting
    xQueueReceive(Q_threshold, &float_data_ptr, portMAX_DELAY);
    *float_data_ptr = (float) analogRead(PIN_THRESHOLD_IN) / MAX_ADC_VAL;
    if(xQueueSend(
      Q_threshold, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("SUPERVISOR: Failed to send Q_threshold.");
    }

    // Update user LED persistence (moving avg weight) setting
    xQueueReceive(Q_moving_avg_weight, &float_data_ptr, portMAX_DELAY);
    *float_data_ptr = (float) analogRead(PIN_MA_RATIO) / MAX_ADC_VAL;
    if(xQueueSend(
      Q_moving_avg_weight, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("SUPERVISOR: Failed to send Q_moving_avg_weight.");
    }

    // Get the stack high water mark and heap stats for debugging
    if (__DEBUG_MEM_SUPER__) {
      debug_mem_to_output(pcTaskGetName(NULL));
    }

    // calculate elapsed ticks and determine delay time to meet execution period
    end_tick = xTaskGetTickCount();
    delay_align_to_period(start_tick, end_tick, PERIOD_MILLIS_USER_INPUT);
  }
}

void Task_SetLEDs(void *pvParameters) {
  // task variables
  CRGB leds_fft[NUM_LEDS_FFT];
  CRGB leds_user_input[NUM_LEDS_USER];
  CRGB scale_palettes[NUM_COLOR_SCALES][NUM_COLORS] = COLOR_SCALES_ARRAY;
  float* float_data_ptr;
  float* fft_output;
  uint8_t* byte_data_ptr;
  uint8_t palette_sel;
  uint8_t palette_index;
  float fft_mags_moving_avg[NUM_LEDS_FFT];
  float weight_ratio;
  float threshold_ratio;
  float brightness_setting;
  float brightness_ratio;
  TickType_t start_tick, end_tick, elapsed_ticks, delay_ticks;
  int i;
  PALETTE_TYPE palette;

  // Initialize FastLED objects
  FastLED.addLeds<WS2812, PIN_LED_FFT_CONT, GRB>(leds_fft, NUM_LEDS_FFT);
  FastLED.addLeds<WS2812, PIN_LED_USER_CONT, RGB>(leds_user_input, NUM_LEDS_USER);

  // Initialize FFT data and LED settings to off-values
  for (i = 0; i < NUM_LEDS_FFT; i++) {
    fft_mags_moving_avg[i] = 0;
    leds_fft[i] = CRGB(0, 0, 0);
  }

  // Initialize user input LEDs to off
  for (i = 0; i < NUM_LEDS_USER; i++) {
    leds_user_input[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  // Steady-state operations
  for (;;) {
    // Time operations to align ending delay with total desired
    // execution period
    start_tick = xTaskGetTickCount();

    // Determine colorscale palette
    xQueueReceive(Q_palette_sel, &byte_data_ptr, portMAX_DELAY);
    palette_sel = *byte_data_ptr;
    if(xQueueSend(
      Q_palette_sel, &byte_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("LED: Failed to send Q_thresQ_palette_selhold");
    }
    for (i = 0; i < NUM_COLORS; i++) {
      palette[i] = scale_palettes[palette_sel][i];
    }

    // Get user input settings and update user input LEDs
    //  leds_user_input[0] -- persistence moving avg weight
    //  leds_user_input[1] -- threshold
    //  leds_user_input[2] -- brightness

    // BRIGHTNESS
    xQueueReceive(Q_brightness, &float_data_ptr, portMAX_DELAY);
    brightness_ratio = *float_data_ptr;
    brightness_setting = brightness_ratio * MAX_FAST_LED_INDEX;
    if(xQueueSend(
      Q_brightness, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("LED: Failed to send Q_brightness");
    }
    leds_user_input[2] = ColorFromPalette(palette, brightness_setting);

    // THRESHOLD
    xQueueReceive(Q_threshold, &float_data_ptr, portMAX_DELAY);
    threshold_ratio = *float_data_ptr;
    if(xQueueSend(
      Q_threshold, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("LED: Failed to send Q_threshold");
    }
    palette_index = (uint8_t) (threshold_ratio * MAX_FAST_LED_INDEX);
    leds_user_input[1] = ColorFromPalette(palette, palette_index);

    // PERSISTENCE WEIGHT
    xQueueReceive(Q_moving_avg_weight, &float_data_ptr, portMAX_DELAY);
    weight_ratio = *float_data_ptr;
    if(xQueueSend(
      Q_moving_avg_weight, &float_data_ptr, portMAX_DELAY) != pdPASS) {
      Serial.println("LED: Failed to send Q_moving_avg_weight");
    }
    palette_index = (uint8_t) (weight_ratio * MAX_FAST_LED_INDEX);
    leds_user_input[0] = ColorFromPalette(palette, palette_index);


    // Update FFT magnitude moving averages and LEDs with newest data
    xQueueReceive(Q_fft_output, &fft_output, portMAX_DELAY);
    vTaskSuspend(T_audio);
    LED_colorscale_set(fft_mags_moving_avg,
                       fft_output, // only give real half of FFT output
                       leds_fft,
                       &palette,
                       NUM_LEDS_FFT,
                       MAX_FAST_LED_INDEX,
                       threshold_ratio,
                       brightness_setting,
                       weight_ratio);
    if(xQueueSend(
      Q_fft_output, &fft_output, portMAX_DELAY) != pdPASS) {
      Serial.println("LED: Failed to send Q_fft_output");
    }
    vTaskResume(T_audio);

    // Update LED outputs
    FastLED.show();

    // Get the stack high water mark and heap stats for debugging
    if (__DEBUG_MEM_EVENT__) {
      debug_mem_to_output(pcTaskGetName(NULL));
    }

    // calculate elapsed ticks and determine delay time to meet execution period
    end_tick = xTaskGetTickCount();
    delay_align_to_period(start_tick, end_tick, PERIOD_MILLIS_LED);
  }
}

void Task_SampleAudio(void *pvParameters) {
  TickType_t start_tick, end_tick, elapsed_ticks;

  // pointer to RA4M1 timer register struct
  R_GPT0_Type* timer_ptr;

  // Create audio and FFT data buffers
  float* audio_data = (float*) pvPortMalloc(NUM_SAMPLES * sizeof(float));
  float fft_intermediate[NUM_SAMPLES];  // buffer for data between FFT comp
                                        // output and complex-->magnitude comp
  float* fft_output;
  float energy_scaling_ratio;

  // set up fft object
  bool ifftFlag = 0;
  arm_rfft_fast_instance_f32 fft;
  arm_rfft_fast_init_f32(&fft, NUM_SAMPLES);

  // Steady-state operations
  for (;;) {
    // Time operations to align ending delay with total desired
    // execution period
    start_tick = xTaskGetTickCount();
    
    // get FFT output buffer
    xQueueReceive(Q_fft_output, &fft_output, portMAX_DELAY);
    
    // Take sample
    sample(audio_data, F_CPU, F_SAMPLE, NUM_SAMPLES, PIN_AUDIO_IN);

    // Print audio data for debugging
    if (__DEBUG_AUDIO__) {
      Serial.println("--------- AUDIO DATA BEGIN ---------\n");
      Serial.print(" Sample Rate: "); Serial.print(F_SAMPLE);
      for (int i = 0; i < NUM_SAMPLES; i++) {
        Serial.print("\n  Audio raw sample #"); Serial.print(i);
        Serial.print("  ----  "); Serial.println(audio_data[i]);
      }
      Serial.println("---------- AUDIO DATA END ----------\n\n");
    }

    // Perform FFT computation
    arm_rfft_fast_f32(&fft, audio_data, fft_intermediate, ifftFlag);

    // Convert complex data to magnitudes
    arm_cmplx_mag_f32(fft_intermediate, fft_output, NUM_SAMPLES_HALF);

    // scale data for Pareseval's theorum (signal energy representation in Freq domain) 
    if (SCALE_FFT_FOR_PARSEVAL) {
      // Calculate FFT scaling ratio from sample rate to maintain correct
      energy_scaling_ratio = 1.0 / F_SAMPLE;

      // NOT scaling DC component since it will not be used (fft_output[0])
      for (int i = 1; i < NUM_SAMPLES_HALF; i++) {
        fft_output[i] *= energy_scaling_ratio;
      }
    }

    // Print FFT magnitude data for debugging
    if (__DEBUG_FFT__) {
      Serial.println("--------- FFT OUTPUT BEGIN ---------\n\n");
      for (int i = 0; i < NUM_SAMPLES_HALF; i++) {
        Serial.print("  FFT Data #"); Serial.print(i);
        Serial.print("  ----  "); Serial.println(fft_output[i]);
      }
      Serial.println("---------- FFT OUTPUT END ----------\n\n");
    }
    
    // Send FFT output data to queue
    if(xQueueSend(
      Q_fft_output, &fft_output, portMAX_DELAY) != pdPASS) {
      Serial.println("FFT Task: Failed to send Q_fft_output.");
    }

    // Get the stack high water mark and heap stats for debugging
    if (__DEBUG_MEM_EVENT__) {
      debug_mem_to_output(pcTaskGetName(NULL));
    }

    // calculate elapsed ticks and determine delay time to meet execution period
    end_tick = xTaskGetTickCount();
    delay_align_to_period(start_tick, end_tick, PERIOD_MILLIS_AUDIO);
  }
}

//// Helper Functions //////////////////////////////////////////////////////////
void debug_mem_to_output(char* task_name) {
  vTaskSuspendAll();
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("("); Serial.print(task_name); Serial.print(")");
  Serial.print(" highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  xHeapStats heapstats;
  vPortGetHeapStats(&heapstats);
  Serial.print("    Largest free heap block -- "); 
  Serial.println(heapstats.xSizeOfLargestFreeBlockInBytes / 4);
  xTaskResumeAll();
}

void delay_align_to_period(TickType_t start_ticks, 
                           TickType_t end_ticks, 
                           int period_millis) {
  TickType_t elapsed_ticks = end_ticks - start_ticks;
  TickType_t delay_ticks 
    = (period_millis / portTICK_PERIOD_MS) - elapsed_ticks;
  if (delay_ticks >= MIN_TICKS_DELAY) {
    if (__DEBUG_DELAYS__) {
      Serial.print("DELAY Invoked: (");
      Serial.print(pcTaskGetName(NULL)); Serial.println(")");
    }

    // add time to allow serial output during debugging
    if (__SERIAL_OUT__) { 
      delay_ticks += TICKS_FOR_SERIAL;
    }

    vTaskDelay(delay_ticks);
    if (__DEBUG_DELAYS__) {
      Serial.print("DELAY Ended: (");
      Serial.print(pcTaskGetName(NULL)); Serial.println(")");
    }
  }
}
