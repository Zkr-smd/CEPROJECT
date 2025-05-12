#include <Arduino.h>
#include <arduinoFFT.h>

#define SAMPLE_RATE 8000
#define FRAME_SIZE 256
#define NUM_FRAMES 61
#define NUM_MFCC 13
#define NUM_FILTERS 26

float audioBuffer[FRAME_SIZE];      // Current frame buffer
float preemph[FRAME_SIZE];          // After pre-emphasis
float windowed[FRAME_SIZE];         // After Hamming window
float real[FRAME_SIZE];             // Real part for FFT
float imag[FRAME_SIZE];             // Imaginary part for FFT
float melEnergies[NUM_FILTERS];     // Output of Mel filter bank
float mfcc[NUM_MFCC];               // Final MFCC output

arduinoFFT FFT = arduinoFFT(real, imag, FRAME_SIZE, SAMPLE_RATE);

// Pre-emphasis filter
void applyPreEmphasis(float* input, float* output) {
  output[0] = input[0];
  for (int i = 1; i < FRAME_SIZE; i++) {
    output[i] = input[i] - 0.97 * input[i - 1];
  }
}

// Apply Hamming window
void applyHamming(float* input, float* output) {
  for (int i = 0; i < FRAME_SIZE; i++) {
    output[i] = input[i] * (0.54 - 0.46 * cos(2 * PI * i / (FRAME_SIZE - 1)));
  }
}

// Dummy MEL filter and DCT implementation (replace with real functions)
void applyMelFilter(float* powerSpec, float* melOutput) {
  for (int i = 0; i < NUM_FILTERS; i++) {
    melOutput[i] = 0;
    for (int j = 1; j < FRAME_SIZE / 2; j++) {
      melOutput[i] += powerSpec[j]; // Simplified, use triangular filter bank
    }
    melOutput[i] = log(melOutput[i] + 1e-6);  // Log energy
  }
}

void computeDCT(float* input, float* output) {
  for (int k = 0; k < NUM_MFCC; k++) {
    output[k] = 0;
    for (int n = 0; n < NUM_FILTERS; n++) {
      output[k] += input[n] * cos(PI * k * (2 * n + 1) / (2.0 * NUM_FILTERS));
    }
  }
}

void computeMFCC(float* input) {
  applyPreEmphasis(input, preemph);
  applyHamming(preemph, windowed);

  for (int i = 0; i < FRAME_SIZE; i++) {
    real[i] = windowed[i];
    imag[i] = 0;
  }

  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  applyMelFilter(real, melEnergies);
  computeDCT(melEnergies, mfcc);
}

void setup() {
  Serial.begin(115200);

  // Simulate 440 Hz sine wave for 1 frame
  for (int i = 0; i < FRAME_SIZE; i++) {
    float t = (float)i / SAMPLE_RATE;
    audioBuffer[i] = 0.6 * sin(2 * PI * 440 * t);
  }

  computeMFCC(audioBuffer);

  Serial.println("MFCCs:");
  for (int i = 0; i < NUM_MFCC; i++) {
    Serial.println(mfcc[i], 4);
  }
}

void loop() {
  // No loop logic in this demo
}
