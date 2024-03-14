// Test code for arduinoFFT library
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <arduinoFFT.h>

#define FFT_NUM_ITERATIONS 5
#define FFT_LENGTH 256
#define FFT_SAMPLE_RATE 65536.0
#define FFT_NOISE_SCALER 256.0

double real[FFT_LENGTH];
double imag[FFT_LENGTH];

void setup() {
  Serial.begin(115200);

  // set up signal with two sine waves
  for (int i = 0; i < FFT_LENGTH; i++) {
    real[i] = 3 * sin((8192.0 * 2.0 * M_PI) / (FFT_SAMPLE_RATE / 2) * i)
            + sin((4096.0 * 2.0 * M_PI) / (FFT_SAMPLE_RATE / 2) * i);
    imag[i] = 0.0;
    // Serial.print("input: "); Serial.println(real[i]);
  }
  // set up fft
  arduinoFFT fft(real, imag, FFT_LENGTH, FFT_SAMPLE_RATE);

  // perform computation
  uint32_t starttime = millis();
  fft.Compute(FFT_FORWARD);
  // get magnitude

  fft.ComplexToMagnitude(real, imag, FFT_LENGTH);
  uint32_t endtime = millis();
  Serial.print("Time to compute FFT mags: "); Serial.println(endtime - starttime);

  // scale results
  for (int i = 0; i < FFT_LENGTH; i++) {
    real[i] /= FFT_LENGTH / 2;
    Serial.print("output at index "); Serial.print(i);
    Serial.print(": "); Serial.println(real[i]);
  }



  int index8192 = (int) (FFT_LENGTH * 8192.0 / (FFT_SAMPLE_RATE / 2));
  int index4096 = (int) (FFT_LENGTH * 4096.0 / (FFT_SAMPLE_RATE / 2));
  Serial.print("Index of 8192: "); Serial.println(index8192);
  Serial.print("Magnitude at 8192: "); Serial.println(real[index8192]);
  Serial.print("Index of 4096: "); Serial.println(index4096);
  Serial.print("Magnitude at 4096: "); Serial.println(real[index4096]);
}

void loop() {
  // put your main code here, to run repeatedly:

}

