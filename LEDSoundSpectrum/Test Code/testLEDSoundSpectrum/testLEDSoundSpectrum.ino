// Test code for FFT and audio sampling integration
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>
#include <FastLED.h>
#include "config.h"
#include "audio_sampling.h"
#include "WS2812b_colorscale.h"

//#define __DEBUG_MEM__ 1

// global FreeRTOS queues and their data variables for task synchronization
QueueHandle_t Q_audio_data_real;
QueueHandle_t Q_audio_data_imag;
QueueHandle_t Q_leds;
QueueHandle_t Q_palette;
QueueHandle_t Q_threshold;
QueueHandle_t Q_sample_rate;
QueueHandle_t Q_brightness;

// FreeRTOS task routine prototypes
void Task_Setup(void *pvParameters);
void Task_SetSampleRate(void *pvParameters);
void Task_SetThreshold(void *pvParameters);
void Task_SampleAudio(void *pvParameters);
void Task_SetLEDs(void *pvParameters);
void Task_PrintFFT(void *pvParameters);
void Task_Delay(void *pvParameters);


void setup() {
  //Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  R_MSTP->MSTPCRD &= ~(1 << 5); // cancel GPT module stop state"

  // Set analog read resolution
  analogReadResolution(ANALOG_READ_RESOLUTION);

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


// Task definitions
void Task_Setup(void *pvParameters) {
  // create audio data queues
  Q_audio_data_real = xQueueCreate(1, sizeof(double*));
  Q_audio_data_imag = xQueueCreate(1, sizeof(double*));

  double* audio_data_real = 
    (double*) pvPortMalloc(NUM_SAMPLES * sizeof(double));
  double* audio_data_imag = 
    (double*) pvPortMalloc(NUM_SAMPLES * sizeof(double));

  if(xQueueSend(
    Q_audio_data_real, &audio_data_real, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_audio_data_real");
  }
  if(xQueueSend(
    Q_audio_data_imag, &audio_data_imag, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_audio_data_imag");
  }

  // create LED data queues
  Q_leds = xQueueCreate(1, sizeof(CRGB*));
  Q_palette = xQueueCreate(1, sizeof(PALETTE_TYPE*));

  // led settings and color palette arrays
  CRGB* leds = (CRGB*) pvPortMalloc(NUM_LEDS * sizeof(CRGB));
  PALETTE_TYPE* color_palette_ptr =
    (PALETTE_TYPE*) pvPortMalloc(sizeof(PALETTE_TYPE));

  // initialize color palette array values
  int colorscale_hex[NUM_COLORS] = PALETTE_ARRAY;
  for (int i = 0; i < NUM_COLORS; i++) {
    (*color_palette_ptr)[i] = CRGB(colorscale_hex[i]);
  }

  // initialize LED array
  FastLED.addLeds<WS2812, PIN_LED_CONT, GRB>(leds, NUM_LEDS);

  // initialize LEDs to off
  for (int32_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = SET_LEDS_OFF;
  }
  FastLED.show();
  
  // place in queues
  if(xQueueSend(
    Q_leds, &leds, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_leds");
  }
  if(xQueueSend(
    Q_palette, &color_palette_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_palette");
  }

  // create threshold data queue
  Q_threshold = xQueueCreate(1, sizeof(double));
  double threshold = 0.5 * (1 << ANALOG_READ_RESOLUTION);
  if(xQueueSend(Q_threshold, &threshold, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_threshold");
  }

  // create sampling rate data queue
  Q_sample_rate = xQueueCreate(1, sizeof(double*));
  double* f_sample = (double*) pvPortMalloc(sizeof(double*));
  if(xQueueSend(Q_sample_rate, &f_sample, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_sample_rate");
  }

  // create brightness data queue
  Q_brightness = xQueueCreate(1, sizeof(uint8_t*));
  uint8_t* brightness = (uint8_t*) pvPortMalloc(sizeof(uint8_t*));
  if(xQueueSend(Q_brightness, &brightness, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_brightness");
  }

  // schedule audio and user-setting tasks
  bool created = false;
  // created = xTaskCreate(Task_SetSampleRate,
  //             "Set Sample Rate Task",
  //             STACK_DEPTH_SAMPLERATE,
  //             NULL,
  //             PRIORITY_SAMPLERATE,
  //             NULL);
  // if (!created) {
  //   Serial.println("Setup: Failed to create Set SampleRate Task");
  // }
  // created = xTaskCreate(Task_SetThreshold,
  //             "Set Threshold Task",
  //             STACK_DEPTH_SET_THRESHOLD,
  //             NULL,
  //             PRIORITY_SET_THRESHOLD,
  //             NULL);
  // if (!created) {
  //   Serial.println("Setup: Failed to create Threshold Task");
  // }
  created = xTaskCreate(Task_SampleAudio,
              "Audio Sampler Task",
              STACK_DEPTH_AUDIO,
              NULL,
              PRIORITY_AUDIO,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Audio Task");
  }     

  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("SETUP: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif. // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}

// void Task_SetSampleRate(void *pvParameters) {
//   // establish delay ticks pointer for task
//   double* f_sample;
//   double f_sample_max = F_SAMPLE_MAX;
//   double ratio = 0.0;  // ratio of analog raed to max read value
//   for(;;) {
//     xQueueReceive(Q_sample_rate, &f_sample, portTICK_PERIOD_MS);
//     // get analog reading and set sample rate
//     ratio = (double) analogRead(PIN_SAMPLE_RATE) / MAX_ADC_VAL;
//     *f_sample = ratio * f_sample_max;
//     if (xQueueSend(Q_sample_rate, &f_sample, portMAX_DELAY) != pdPASS) {
//       Serial.println("Failed to send data sample rate.");
//     }

//     // Get the high water mark for debugging
//     #ifdef __DEBUG_MEM_STAT__
//     UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
//     Serial.print("SET SAMPLE RATE: highwatermark (32-bit words): ");
//     Serial.println(uxHighWaterMark);
//     #endif  // __DEBUG_MEM_STAT__

//     // Delay minimize CPU usage used waiting on human inputs
//     vTaskDelay(USER_SET_SAMPLE_MILLIS / portTICK_PERIOD_MS);
//   }
// }

// void Task_SetThreshold(void *pvParameters) {
//   // Establish threshold variable
//   double threshold_val;

//   // steady-state operation
//   for(;;) {
//     xQueueReceive(Q_threshold, &threshold_val, portTICK_PERIOD_MS);
//     threshold_val = (double) analogRead(PIN_THRESHOLD_IN) / MAX_ADC_VAL;
//     if(xQueueSend(Q_threshold, &threshold_val, portTICK_PERIOD_MS) != pdPASS) {
//       Serial.println("Threshold Task: Failed to send threshold data.");
//     }

//     // Get the high water mark for debugging
//     #ifdef __DEBUG_MEM_STAT__
//     UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
//     Serial.print("SET THRESHOLD: highwatermark (32-bit words): ");
//     Serial.println(uxHighWaterMark);
//     #endif  // __DEBUG_MEM_STAT__

//     // Delay minimize CPU usage used waiting on human inputs
//     vTaskDelay(USER_SET_THRESH_MILLIS / portTICK_PERIOD_MS);
//   }
// }

void Task_SampleAudio(void *pvParameters) {
  // Create local pointers to queued data variables
  double* audio_data_real_ptr;
  double* audio_data_imag_ptr;
  double* f_sample;
  double f_sample_setting;
  double f_sample_max = F_SAMPLE_MAX;
  double ratio = 0.0; // ratio of analog raed to max read value

  // pointer toRA4M1 timer register struct
  R_GPT0_Type* timer_ptr;

  // Get sample rate setting
  xQueueReceive(Q_sample_rate, &f_sample, portMAX_DELAY);
  ratio = (double) analogRead(PIN_SAMPLE_RATE) / MAX_ADC_VAL;
  f_sample_setting = ratio * f_sample_max;
  *f_sample = f_sample_setting;
  if (xQueueSend(Q_sample_rate, &f_sample, portMAX_DELAY) != pdPASS) {
    Serial.println("Audio: Failed to send sample rate data.");
  }

  // Wait for use of audio data arrays and sample rate data
  xQueueReceive(Q_audio_data_imag, &audio_data_imag_ptr, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, &audio_data_real_ptr, portMAX_DELAY);

  // Take sample
  sample(audio_data_real_ptr, F_CPU, f_sample_setting, NUM_SAMPLES, PIN_AUDIO_IN);

  // print out Audio data
  #ifdef __DEBUG_AUDIO__
  Serial.println("--------- AUDIO DATA BEGIN ---------\n");
   Serial.print(" Sample Rate: "); Serial.print(f_sample_setting);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print("\n  Audio raw sample #"); Serial.print(i);
    Serial.print("  ----  "); Serial.println(audio_data_real_ptr[i]);
  }
  Serial.println("---------- AUDIO DATA END ----------\n\n");
  #endif  // __DEBUG_AUDIO__

  // Schedule FFT task
  int task_created = xTaskCreate(Task_FFT,
                                  "FFT Task",
                                  STACK_DEPTH_FFT,
                                  NULL,
                                  PRIORITY_FFT,
                                  NULL);

  // Send use of data array to successfully scheduled FFT task
  if (!task_created) {
    Serial.println("Audio Task: Failed to create FFT Task");
  }

  if(xQueueSend(
    Q_audio_data_real, (void*) &audio_data_real_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("Audio Task: Failed to send real audio data.");
  }
  if(xQueueSend(
    Q_audio_data_imag, (void*) &audio_data_imag_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("Audio Task: Failed to send imag audio data.");
  }

  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("AUDIO: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif  // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}

void Task_FFT(void *pvParameters) {
  // Initialize data variables ptrs for arduinoFFT object
  double* audio_data_real_ptr;
  double* audio_data_imag_ptr;
  double* f_sample;
  double f_sample_setting;

  // Get sample rate setting
  xQueueReceive(Q_sample_rate, &f_sample, portMAX_DELAY);
  f_sample_setting = *f_sample;
  if (xQueueSend(Q_sample_rate, &f_sample, portMAX_DELAY) != pdPASS) {
    Serial.println("FFT: Failed to send sample rate data.");
  }

  // Wait for audio data
  xQueueReceive(Q_audio_data_imag, (void*) &audio_data_imag_ptr, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, (void*) &audio_data_real_ptr, portMAX_DELAY);

  // Zero out imaginary array
  for (int i = 0; i < NUM_SAMPLES; i++) {
    audio_data_imag_ptr[i] = 0.0;
  }

  // Initialize arduinoFFT object and remove DC component from data
  arduinoFFT fft(audio_data_real_ptr, audio_data_imag_ptr, 
                NUM_SAMPLES, f_sample_setting);
  fft.DCRemoval();

  // Perform in-place FFT on data arrays and in-place convert to magnitude data
  fft.Compute(FFT_FORWARD);
  fft.ComplexToMagnitude(audio_data_real_ptr, audio_data_imag_ptr, NUM_SAMPLES);
  
  // Schedule FFT LED setting task
  int task_created = xTaskCreate(Task_SetLEDs,
              "LED-Setting Task",
              STACK_DEPTH_LED,
              NULL,
              PRIORITY_LED,
              NULL);
  
  if (!task_created) {
    Serial.println("FFT Task: Failed to create LED Task");
  } 

  // Send computed FFT audio data to queues
  if(xQueueSend(
    Q_audio_data_real, (void*) &audio_data_real_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("FFT Task: Failed to send real audio data.");
  }
  if(xQueueSend(
    Q_audio_data_imag, (void*) &audio_data_imag_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("FFT Task: Failed to send imag audio data.");
  }
  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("FFT: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif  // __DEBUG_MEM_EVENT__

  // Halt task
  vTaskDelete(NULL);
}

void Task_SetLEDs(void *pvParameters) {
  double* magnitudes;
  double* unused_imag;
  double* threshold_val;
  double threshold_setting;
  double* brightness_val;
  double brightness_setting;
  double brightness_ratio;
  CRGB* led_settings;
  PALETTE_TYPE* color_palette_ptr;

  // Wait for audio data from FFT
  xQueueReceive(Q_audio_data_imag, &unused_imag, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, &magnitudes, portMAX_DELAY);

  // Wait for use of LED and color palette arrays
  xQueueReceive(Q_leds, &led_settings, portMAX_DELAY);
  xQueueReceive(Q_palette, &color_palette_ptr, portMAX_DELAY);

  // Wait for use of brightness data
  xQueueReceive(Q_brightness, &brightness_val, portMAX_DELAY);

  // Set brightness from analog input
  brightness_ratio = (double) analogRead(PIN_BRIGHTNESS_IN) / MAX_ADC_VAL;
  brightness_setting = brightness_ratio * MAX_FAST_LED_INDEX;
  *brightness_val = brightness_setting;

  // release use of brightness data
  if(xQueueSend(
    Q_brightness, &brightness_val, portMAX_DELAY) != pdPASS) {
    Serial.println("LED Task: Failed to send brightness data");
  }

  threshold_setting = (double) analogRead(PIN_THRESHOLD_IN) / MAX_ADC_VAL;
  magnitudes[0] = magnitudes[1]; // since DC conponent was moved

  // set LEDS
  LED_colorscale_set32(magnitudes,
                        led_settings,
                        color_palette_ptr,
                        NUM_LEDS,
                        MAX_FAST_LED_INDEX,
                        DATA_LED_SCALE,
                        threshold_setting,
                        brightness_setting);

  // print out synthetic LED color index data
  #ifdef __DEBUG_LED__
  Serial.println("--------- LED COLORSCALE INDEX BEGIN ---------\n");
  for (int i = 0; i < NUM_LEDS; i++) {
    Serial.print("  LED Setting mag: "); Serial.print(i);
    Serial.print("  ----  "); 
    Serial.println(magnitudes[i]);
  }
  Serial.println("---------- LED COLORSCALE INDEX END ----------\n\n");
  #endif  // __DEBUG_LED__

  // release use of LED and color palette arrays
  if(xQueueSend(
    Q_leds, &led_settings, portMAX_DELAY) != pdPASS) {
    Serial.println("LED Task: Failed to send leds array ptr");
  }
  if(xQueueSend(
    Q_palette, &color_palette_ptr, portMAX_DELAY) != pdPASS) {
    Serial.println("LED: Failed to send color_palette_ptr");
  }

  // release use of audio data arrays
  if(xQueueSend(
    Q_audio_data_real, (void*) &magnitudes, portMAX_DELAY) != pdPASS) {
    Serial.println("LED Task: Failed to send real audio data.");
  }
  if(xQueueSend(
    Q_audio_data_imag, (void*) &unused_imag, portMAX_DELAY) != pdPASS) {
    Serial.println("LED Task: Failed to send imag audio data.");
  }

  #ifdef __USE_DELAY__
  // Initialize delay task
  bool created = xTaskCreate(Task_Delay,
              "Delay Task",
              STACK_DEPTH_DELAY,
              NULL,
              PRIORITY_DELAY,
              NULL);
  if (!created) {
    Serial.println("LED Task: Failed to create Delay Task");
  }
  #endif  // __USE_DELAY__

  #ifndef __USE_DELAY__
  // Initialize delay task

  // initialize audio task and halt
  bool created = xTaskCreate(Task_SampleAudio,
              "Audio Sampler Task",
              STACK_DEPTH_AUDIO,
              NULL,
              PRIORITY_AUDIO,
              NULL);
  if (!created) {
    Serial.println("LED Task: Failed to create Audio Task");
  }
  #endif  // __USE_DELAY__

  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("LED: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif  // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}

void Task_Delay(void *pvParameters) {
  // delay value variable
  uint32_t delay_setting = DEFAULT_DELAY_MILLIS;

  // Invoke schedule iteration delay
  vTaskDelay((double) delay_setting / portTICK_PERIOD_MS);

  // initialize audio task and halt
  bool created = xTaskCreate(Task_SampleAudio,
              "Audio Sampler Task",
              STACK_DEPTH_AUDIO,
              NULL,
              PRIORITY_AUDIO,
              NULL);
  if (!created) {
    Serial.println("Delay Task: Failed to create Audio Task");
  }

  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("DELAY: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif  // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}
