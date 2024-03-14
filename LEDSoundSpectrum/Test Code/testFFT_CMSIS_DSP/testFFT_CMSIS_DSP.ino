// Test code for Arm CMSIS DSP FFT library
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <arm_math.h>
#include <Arduino_FreeRTOS.h>

#define FFT_LENGTH 256
#define FFT_SAMPLE_RATE 65536.0

#define STACK_DEPTH_SETUP 128
#define STACK_DEPTH_AUDIO 128
#define STACK_DEPTH_FFT 384
#define PRIORITY_SETUP 2
#define PRIORITY_FFT 1
#define PRIORITY_AUDIO 0

QueueHandle_t Q_audio_data;
QueueHandle_t Q_fft_output;

void Task_Setup(void *pvParameters) {
  // create audio data and FFT output queues
  Q_audio_data = xQueueCreate(1, sizeof(float*));
  Q_fft_output = xQueueCreate(1, sizeof(float*));

  float* audio_data = 
    (float*) pvPortMalloc(FFT_LENGTH * sizeof(float));
  float* fft_output = 
    (float*) pvPortMalloc(FFT_LENGTH * sizeof(float));

  if(xQueueSend(
    Q_audio_data, &audio_data, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_audio_data");
  }
  if(xQueueSend(
    Q_fft_output, &fft_output, portMAX_DELAY) != pdPASS) {
    Serial.println("Failed to allocate Q_fft_output");
  }


  // launch audio sampling task
  bool created = false;
  created = xTaskCreate(Task_SampleAudio,
              "Audio Sampler Task",
              STACK_DEPTH_AUDIO,
              NULL,
              PRIORITY_AUDIO,
              NULL);
  if (!created) {
    Serial.println("Setup: Failed to create Audio Task");
  }     
  vTaskDelete(NULL);
}

void Task_SampleAudio(void *pvParameters) {
  float* audio_data;

  // Wait for use of audio data arrays and sample rate data
  xQueueReceive(Q_audio_data, &audio_data, portMAX_DELAY);

  // Populate data buffer with signal
  for (int i = 0; i < FFT_LENGTH; i++) {
    audio_data[i] = 3 * sin((8192.0 * 2.0 * M_PI) / (FFT_SAMPLE_RATE / 2) * i)
            + sin((4096.0 * 2.0 * M_PI) / (FFT_SAMPLE_RATE / 2) * i);
    // Serial.print("input: "); Serial.println(real[i]);
  }
  Serial.println("Audio signal generated..");

  // send audio data to queue
  if(xQueueSend(
    Q_audio_data, &audio_data, portMAX_DELAY) != pdPASS) {
    Serial.println("AUDIO: Failed to send Q_audio_data");
  }
  int task_created = xTaskCreate(Task_FFT,
                                  "FFT Task",
                                  STACK_DEPTH_FFT,
                                  NULL,
                                  PRIORITY_FFT,
                                  NULL);
  if (!task_created) {
    Serial.println("AUDIO: Failed to create FFT Task");
  }
  vTaskDelete(NULL);
}

void Task_FFT(void *pvParameters) {
  float fft_intermediate[FFT_LENGTH];
  float* audio_data;
  float* fft_output;

  // Wait for use of audio data arrays and fft output array
  xQueueReceive(Q_audio_data, &audio_data, portMAX_DELAY);
  xQueueReceive(Q_fft_output, &fft_output, portMAX_DELAY);

  // set up fft
  arm_rfft_fast_instance_f32 fft;
  bool ifftFlag = 0;
  arm_rfft_fast_init_f32(&fft, FFT_LENGTH);

  // perform computation
  uint32_t starttime = millis();
  arm_rfft_fast_f32(&fft, audio_data, fft_intermediate, ifftFlag);

  // get magnitude
  arm_cmplx_mag_f32(fft_intermediate, fft_output, FFT_LENGTH >> 1);

  uint32_t endtime = millis();

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("FFT: highwatermark (32-bit words): ");
  Serial.println(uxHighWaterMark);
  Serial.print("Time to compute FFT mags: "); Serial.println(endtime - starttime);
  // Serial.println("---- FFT Output Begin ----");
  // for (int i = 0; i < FFT_LENGTH >> 1; i++) {
  //   Serial.print("output at index "); Serial.print(i);
  //   Serial.print(": "); Serial.println(fft_output[i]);
  // }
  // Serial.println("---- FFT Output End ----");
  
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);

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

