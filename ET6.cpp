#include <arduinoFFT.h>
#include "arduinoFFT.h"

// ─────── Configuration ───────
#define AUDIO_LENGTH   8000    // 1 s @ 8 kHz
#define FRAME_SIZE     256
#define HOP_SIZE       128
#define NUM_FRAMES     ((AUDIO_LENGTH - FRAME_SIZE) / HOP_SIZE + 1)
#define NUM_MFCC       13
#define SAMPLE_RATE    8000    // Hz

// ─────── Audio Buffer ───────
// Paste the full array you generated here, or use #include if you put it in a .h file
#include "audioBuffer_8000_float_array.h"  
// which should define:
// float audioBuffer[8000] = { 0.00000, 0.20324, … , -0.00123 };


// ─────── Intermediate Buffers ───────
float frames[NUM_FRAMES][FRAME_SIZE];  // Overlapping frames

// For MFCC extraction
arduinoFFT FFT = arduinoFFT();
float fftInput[FRAME_SIZE];
float fftOutput[FRAME_SIZE];
float spectrum[FRAME_SIZE/2];
float melEnergies[NUM_MFCC];
float mfccs[NUM_MFCC];


// ─────── Function Prototypes ───────
void frameAudio();
void preprocessFrame(float *frame);
void computeFFT(float *frame);
void applyMelFilterBank(float *spectrum, float *melEnergies);
void computeDCT(float *melEnergies, float *mfccsOut);
void extractMFCCs(float *inFrame, float *outMFCCs);
void printMFCCs(int frameIndex);


// ─────── Arduino Setup ───────
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1) Split into overlapping frames
  frameAudio();

  // 2) Extract and print MFCCs for the first frame as validation
  extractMFCCs(frames[0], mfccs);
  printMFCCs(0);
}

void loop() {
  // nothing here – you can iterate over all frames if you like
}


// ─────── ET5: Frame the Signal ───────
void frameAudio() {
  for (int f = 0; f < NUM_FRAMES; f++) {
    int startIdx = f * HOP_SIZE;
    for (int i = 0; i < FRAME_SIZE; i++) {
      frames[f][i] = audioBuffer[startIdx + i];
    }
  }
}


// ─────── ET6 Step 1: Pre-emphasis + Hamming Window ───────
void preprocessFrame(float *frame) {
  // Pre-emphasis
  const float alpha = 0.97;
  for (int i = FRAME_SIZE - 1; i > 0; i--) {
    frame[i] = frame[i] - alpha * frame[i - 1];
  }
  frame[0] *= (1 - alpha);

  // Hamming window
  for (int i = 0; i < FRAME_SIZE; i++) {
    float w = 0.54 - 0.46 * cos(2 * PI * i / (FRAME_SIZE - 1));
    frame[i] *= w;
  }
}


// ─────── ET6 Step 2: FFT → Magnitude Spectrum ───────
void computeFFT(float *frame) {
  // prepare real/imag
  for (int i = 0; i < FRAME_SIZE; i++) {
    fftInput[i] = frame[i];
    fftOutput[i] = 0.0;
  }
  // no additional window here since already applied
  FFT.Windowing(fftInput, FRAME_SIZE, FFT_WIN_TYP_NONE, FFT_FORWARD);
  FFT.Compute(fftInput, fftOutput, FRAME_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(fftInput, fftOutput, spectrum, FRAME_SIZE);
}


// ─────── ET6 Step 3: Mel Filter Bank (simplified) ───────
void applyMelFilterBank(float *spectrum, float *melEnergies) {
  // very coarse triangular filters over first FRAME_SIZE/2 bins
  int bins = FRAME_SIZE / 2;
  for (int m = 0; m < NUM_MFCC; m++) {
    float energy = 0.0;
    // define a small band per MFCC (here 4 bins wide per filter)
    int start = m * 2;
    int peak  = start + 2;
    int end   = start + 4;
    for (int i = start; i < end && i < bins; i++) {
      float weight = (i <= peak)
        ? (float)(i - start) / (peak - start)
        : (float)(end - i)   / (end - peak);
      energy += spectrum[i] * weight;
    }
    melEnergies[m] = log(energy + 1e-6);  // log compression
  }
}


// ─────── ET6 Step 4: DCT Type II ───────
void computeDCT(float *melEnergies, float *mfccsOut) {
  for (int k = 0; k < NUM_MFCC; k++) {
    float sum = 0.0;
    for (int n = 0; n < NUM_MFCC; n++) {
      sum += melEnergies[n] * cos(PI * k * (2 * n + 1) / (2.0 * NUM_MFCC));
    }
    mfccsOut[k] = sum;
  }
}


// ─────── ET6: Wrap It All ───────
void extractMFCCs(float *inFrame, float *outMFCCs) {
  // Make a temporary copy for windowing/FFT
  float temp[FRAME_SIZE];
  memcpy(temp, inFrame, FRAME_SIZE * sizeof(float));

  preprocessFrame(temp);
  computeFFT(temp);
  applyMelFilterBank(spectrum, melEnergies);
  computeDCT(melEnergies, outMFCCs);
}


// ─────── Utility: Print MFCCs ───────
void printMFCCs(int frameIndex) {
  Serial.print("MFCCs for frame ");
  Serial.println(frameIndex);
  for (int i = 0; i < NUM_MFCC; i++) {
    Serial.print("  c[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(mfccs[i], 4);
  }
}
