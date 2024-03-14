// Test code for FFT and audio sampling integration
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>
#include "config.h"
#include "audio_sampling.h"

//#define __DEBUG_MEM__ 1

// global FreeRTOS queues and their data variables for task synchronization
QueueHandle_t Q_audio_data_real;
QueueHandle_t Q_audio_data_imag;
QueueHandle_t Q_threshold;
QueueHandle_t Q_sample_rate;

// FreeRTOS task routine prototypes
void Task_Setup(void *pvParameters);
void Task_SetSampleRate(void *pvParameters);
void Task_SetThreshold(void *pvParameters);
void Task_SampleAudio(void *pvParameters);
void Task_FFT(void *pvParameters);
void Task_PrintFFT(void *pvParameters);
void Task_Delay(void *pvParameters);

// Task definitions
void Task_Setup(void *pvParameters) {
  // create audio data queues
  Q_audio_data_real = xQueueCreate(1, sizeof(double*));
  Q_audio_data_imag = xQueueCreate(1, sizeof(double*));

  double* audio_data_real = (double*) pvPortMalloc(NUM_SAMPLES * sizeof(double));
  double* audio_data_imag = (double*) pvPortMalloc(NUM_SAMPLES * sizeof(double));

  for (int i = 0; i < NUM_SAMPLES; i++) {
    audio_data_real[i] = 0;
    audio_data_imag[i] = 0;
  }
  if(xQueueSend(
    Q_audio_data_real, &audio_data_real, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_audio_data_real");
  }
  if(xQueueSend(
    Q_audio_data_imag, &audio_data_imag, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_audio_data_imag");
  }

  // create threshold data queue
  Q_threshold = xQueueCreate(1, sizeof(int16_t));
  int16_t threshold = 0.5 * (1 << ANALOG_READ_RESOLUTION_THRESHOLD);
  if(xQueueSend(Q_threshold, &threshold, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_threshold");
  }

  // create sampling rate data queue
  Q_sample_rate = xQueueCreate(1, sizeof(double));
  double* f_sample = pvPortMalloc(sizeof(double));
  if (f_sample == NULL) {
    Serial.println("SETUP: failed to allocate f_sample variable to heap");
  }
  *f_sample = DEFAULT_SAMPLE_RATE;
  if(xQueueSend(Q_sample_rate, &f_sample, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_sample_rate");
  }

  // schedule audio and user-setting tasks
  bool created = false;
  created = xTaskCreate(Task_SetSampleRate,
              "Set Sample-Rate Task",
              STACK_DEPTH_DELAY,
              NULL,
              PRIORITY_SET_DELAY,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Set Delay Task");
  }
  created = xTaskCreate(Task_SetThreshold,
              "Set Threshold Task",
              STACK_DEPTH_SET_THRESHOLD,
              NULL,
              PRIORITY_SET_THRESHOLD,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Threshold Task");
  }      
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



void Task_SetSampleRate(void *pvParameters) {
  // establish delay ticks pointer for task
  uint32_t delay;
  for(;;) {
    xQueueReceive(Q_delay, &delay, portTICK_PERIOD_MS);
    uint32_t readValue = DEFAULT_DELAY_MILLIS; //Serial.parseInt(); // Parse the incoming integer

    // Validate the parsed integer
    if (readValue > 0) {

      // If valid, send the read value to the queue
      delay = readValue;
    }

    if (!xQueueSend(Q_delay, &delay, portMAX_DELAY) == pdPASS) {
      Serial.println("Delay Task: Failed to send delay data.");
    }

    // Delay minimize CPU usage used waiting on human inputs
    vTaskDelay(USER_SET_DELAY_MILLIS / portTICK_PERIOD_MS);

    // Get the high water mark for debugging
    #ifdef __DEBUG_MEM_STAT__
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("SET DELAY: highwatermark (32-bit words): ");
    Serial.println(uxHighWaterMark);
    #endif  // __DEBUG_MEM_STAT__
    vTaskDelete(NULL);
  }
}

void Task_SetThreshold(void *pvParameters) {
  // Establish threshold variable
  int16_t threshold_val;

  // Shift value to affect scaling to audio resolution
  int16_t shift = 
    ANALOG_READ_RESOLUTION_AUDIO - ANALOG_READ_RESOLUTION_THRESHOLD;

  // steady-state operation
  for(;;) {
    xQueueReceive(Q_threshold, &threshold_val, portTICK_PERIOD_MS);
    threshold_val = (int16_t) (analogRead(PIN_THRESHOLD_IN) << shift);
    if(xQueueSend(Q_threshold, &threshold_val, portTICK_PERIOD_MS) != pdPASS) {
      Serial.println("Threshold Task: Failed to send threshold data.");
    }

    // Delay minimize CPU usage used waiting on human inputs
    vTaskDelay(USER_SET_DELAY_MILLIS / portTICK_PERIOD_MS);

    // Get the high water mark for debugging
    #ifdef __DEBUG_MEM_STAT__
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("SET THRESHOLD: highwatermark (32-bit words): ");
    Serial.println(uxHighWaterMark);
    #endif  // __DEBUG_MEM_STAT__
  }
}

void Task_SampleAudio(void *pvParameters) {
  // Create local pointers to queued data variables
  double* audio_data_real_ptr;
  double* audio_data_imag_ptr;
  int16_t threshold_val;
  int16_t threshold_setting;

  // Configure for sampling
  R_GPT0_Type* timer_ptr;

  // Wait for use of audio data arrays
  xQueueReceive(Q_audio_data_imag, &audio_data_imag_ptr, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, &audio_data_real_ptr, portMAX_DELAY);

  // Wait for use of threshold data (locks threshold task's use of data)
  xQueueReceive(Q_threshold, &threshold_val, portMAX_DELAY);
  threshold_setting = threshold_val;
  xQueueSend(Q_threshold, &threshold_val, portMAX_DELAY); // release

  // Take sample
  sample(audio_data_real_ptr, F_CPU, F_SAMPLE, NUM_SAMPLES, PIN_AUDIO_IN,
            ANALOG_READ_RESOLUTION_AUDIO);

  // for (int i = 0; i < NUM_SAMPLES; i++) {
  //   Serial.print("Sound data #"); Serial.print(i); Serial.print(":  "); 
  //   Serial.println(audio_data_real_ptr[i]);
  // }

  ///////////////////////////////////////////////////////////////////////////////////////
  // IMPLEMENT / DEBUG later

  // // Set audio object's threshold value
  // audio.setBaseLine(threshold_setting);

  // // Take audio sample
  // audio.sample(audio_data_real_ptr, NUM_SAMPLES);
  ///////////////////////////////////////////////////////////////////////////////////////

  // // print out Audio data
  // Serial.println("--------- AUDIO DATA BEGIN ---------\n");
  // for (int i = 0; i < NUM_SAMPLES; i++) {
  //   Serial.print(i);     Serial.print(")  "); 
  //   Serial.print("  Audio raw #"); 
  //   Serial.print((long) ((i * 0.5 * F_SAMPLE) / NUM_SAMPLES));
  //   Serial.print("  ----  "); Serial.println(audio_data_real_ptr[i]);
  // }
  // Serial.println("---------- AUDIO DATA END ----------\n\n");

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

  // Wait for audio data
  xQueueReceive(Q_audio_data_imag, (void*) &audio_data_imag_ptr, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, (void*) &audio_data_real_ptr, portMAX_DELAY);

  // Zero out imaginary array
  for (int i = 0; i < NUM_SAMPLES; i++) {
    audio_data_imag_ptr[i] = 0.0;
  }

  // Initialize arduinoFFT object and remove DC component from data
  arduinoFFT fft(audio_data_real_ptr, audio_data_imag_ptr, 
                NUM_SAMPLES, F_SAMPLE);
  fft.DCRemoval();

  // Perform in-place FFT on data arrays and in-place convert to magnitude data
  fft.Compute(FFT_FORWARD);
  fft.ComplexToMagnitude(audio_data_real_ptr, audio_data_imag_ptr, NUM_SAMPLES);

  // Schedule FFT print task
  int task_created = xTaskCreate(Task_PrintFFT,
              "FFT Print Task",
              STACK_DEPTH_PrintFFT,
              NULL,
              PRIORITY_PRINT,
              NULL);
  
  if (!task_created) {
    Serial.println("FFT Task: Failed to create Print Task");
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

void Task_PrintFFT(void *pvParameters) {
  // Wait for audio data from FFT
  double* magnitudes;
  double* unused_imag;
  xQueueReceive(Q_audio_data_imag, (void*) &unused_imag, portMAX_DELAY);
  xQueueReceive(Q_audio_data_real, (void*) &magnitudes, portMAX_DELAY);

  // print out FFT data
  Serial.println("--------- FFT DATA BEGIN ---------\n");
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print(i);     Serial.print(")  "); 
    Serial.print("  Freq: "); 
    Serial.print((long) ((i * 0.5 * F_SAMPLE) / NUM_SAMPLES));
    Serial.print("  ----  "); Serial.println(magnitudes[i]);
  }
  Serial.println("---------- FFT DATA END ----------\n\n");

  // release use of audio data arrays
  if(xQueueSend(
    Q_audio_data_real, (void*) &magnitudes, portMAX_DELAY) != pdPASS) {
    Serial.println("FFT Task: Failed to send real audio data.");
  }
  if(xQueueSend(
    Q_audio_data_imag, (void*) &unused_imag, portMAX_DELAY) != pdPASS) {
    Serial.println("FFT Task: Failed to send imag audio data.");
  }

  // Initialize delay task
  bool created = xTaskCreate(Task_Delay,
              "Delay Task",
              STACK_DEPTH_DELAY,
              NULL,
              PRIORITY_DELAY,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Audio Task");
  }

  // Get the high water mark for debugging
  #ifdef __DEBUG_MEM_EVENT__
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("PRINT: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  #endif  // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}

void Task_Delay(void *pvParameters) {
  // delay value variable
  uint32_t delay_val;
  uint32_t delay_setting;

  // wait for use of delay data
  xQueueReceive(Q_delay, &delay_val, portMAX_DELAY);
  delay_setting = delay_val;

  // Release delay data
  if(xQueueSend(Q_delay, &delay_val, portMAX_DELAY) != pdPASS) {
    Serial.println("Delay Task: Failed to send delay data.");
  }

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
  Serial.println(uxHighWaterMark); // Convert to bytes
  #endif  // __DEBUG_MEM_EVENT__

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  R_MSTP->MSTPCRD &= ~(1 << 5); // cancel GPT module stop state"

  // Launch setup task
  if(xTaskCreate(Task_Setup,
              "Setup Task",
              STACK_DEPTH_SETUP,
              NULL,
              PRIORITY_SETUP,
              NULL) != pdPASS) {
    Serial.println("Failed to create setup task");        
  };   


  // Start scheduler
  vTaskStartScheduler() ;
}

void loop() {}
