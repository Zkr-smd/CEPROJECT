#include <Arduino.h>
#include "arduinoMFCC.h"

#define FRAME_SIZE 256
#define HOP_SIZE 128
#define MFCC_SIZE 13
#define FILTER_TAP_NUM 21
#define BUFFER_SIZE 512

const int samplingFrequency = 16000;

// MFCC
arduinoMFCC mfcc(FRAME_SIZE, MFCC_SIZE, FRAME_SIZE, samplingFrequency);
float frame[FRAME_SIZE];
float mfccs[MFCC_SIZE];

// RIF FILTER COEFFICIENTS
double filter_taps[FILTER_TAP_NUM] = {
  -0.06072852230688862, -0.009743879979818206, 0.03554309359439512,
  -0.06453910915450291, 0.07912618274311577, -0.06366899994206501,
  0.012191671871374815, 0.06710767199756262, -0.15331654700903166,
  0.21994704013789626, 0.7550762235546863, 0.21994704013789626,
  -0.15331654700903166, 0.06710767199756262, 0.012191671871374815,
  -0.06366899994206501, 0.07912618274311577, -0.06453910915450291,
  0.03554309359439512, -0.009743879979818206, -0.06072852230688862
};

// Buffers
uint16_t adc_buffer[BUFFER_SIZE];
float filtered_buffer[BUFFER_SIZE];
volatile bool data_ready = false;

int position = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Résolution 12 bits
  mfcc.create_hamming_window();
  mfcc.create_mel_filter_bank();
  mfcc.create_dct_matrix();
}

// Simule une lecture ADC
uint16_t readMic() {
  return analogRead(A0);
}

// Applique le filtre RIF au signal
float applyFIR(int pos) {
  float result = 0.0;
  for (int i = 0; i < FILTER_TAP_NUM; i++) {
    int index = (pos - i + BUFFER_SIZE) % BUFFER_SIZE;
    result += filter_taps[i] * adc_buffer[index];
  }
  return result;
}

void loop() {
  // 1. Acquisition
  adc_buffer[position] = readMic();

  // 2. Filtrage
  filtered_buffer[position] = applyFIR(position);

  // 3. Remplissage du frame si possible
  if (position >= FRAME_SIZE) {
    for (int i = 0; i < FRAME_SIZE; i++) {
      frame[i] = filtered_buffer[(position - FRAME_SIZE + i + BUFFER_SIZE) % BUFFER_SIZE];
    }

    // 4. MFCC
    mfcc.compute(frame, mfccs);

    // 5. Affichage des MFCC
    Serial.println("MFCC:");
    for (int i = 0; i < MFCC_SIZE; i++) {
      Serial.print("C"); Serial.print(i); Serial.print(": ");
      Serial.println(mfccs[i], 4);
    }
    Serial.println("=====");
    delay(500);  // attendre un peu
  }

  // Incrément circulaire
  position = (position + 1) % BUFFER_SIZE;
}
