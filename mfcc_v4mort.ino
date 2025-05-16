#include <Arduino.h>
#include "arduinoMFCC.h"

#define BUFFER_SIZE         45
#define ADC_CH              7
#define DAC_CHANNEL         1
#define PinTimer            2
#define TIMER_RC            328
#define DS_FACTOR           4

#define SAMPLE_BUFFER_SIZE 8000
#define FRAME_LENGTH        256
#define HOP_SIZE            (FRAME_LENGTH / 2)

#define MFCC_SIZE           13
#define DCT_MFCC_SIZE       13
#define FREQ_ECH            8000
#define PRE_EMPHASIS_ALPHA  0.97

static const int16_t filter_taps[BUFFER_SIZE] = {
   731,  578,  527,  248, -213, -717, -1074, -1126,  -822,  -260,
   326,  662,  548,  -33, -870, -1576, -1722, -1009,   590,  2784,
  5030, 6708, 7329, 6708, 5030, 2784,   590, -1009, -1722, -1576,
  -870,  -33,  548,  662,   326,  -260,  -822, -1126, -1074,  -717,
  -213,  248,  527,  578,   731
};

volatile uint16_t circBuffer[BUFFER_SIZE];
volatile uint8_t bufIndex = 0;

volatile uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
volatile uint16_t sampleCount = 0;
volatile bool     recordReady = false;

arduinoMFCC mymfcc(MFCC_SIZE, DCT_MFCC_SIZE, FRAME_LENGTH, FREQ_ECH);

volatile uint8_t dsCount = 0;

void setupTimer() {
  PMC->PMC_PCER0 |= (1 << ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2;
  TC0->TC_CHANNEL[0].TC_RC  = TIMER_RC;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
  TC_Start(TC0, 0);
}

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_TRGEN_DIS | ADC_MR_PRESCAL(10) | ADC_MR_STARTUP_SUT96 | ADC_MR_TRACKTIM(3);
  ADC->ADC_CHER = (1u << ADC_CH);
}

void setupDAC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  DACC->DACC_MR = DACC_MR_TRGEN_DIS | DACC_MR_WORD_HALF | DACC_MR_USER_SEL_CHANNEL1 |
                  DACC_MR_REFRESH(1) | DACC_MR_STARTUP_8 | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH1;
  NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;
  digitalWrite(PinTimer, !digitalRead(PinTimer));

  ADC->ADC_CR = ADC_CR_START;
  while ((ADC->ADC_ISR & (1u << ADC_CH)) == 0);
  uint16_t sample = ADC->ADC_CDR[ADC_CH] & 0x0FFF;

  circBuffer[bufIndex] = sample;
  int32_t acc = 0;
  uint8_t idx = bufIndex;
  for (uint8_t t = 0; t < BUFFER_SIZE; t++) {
    acc += filter_taps[t] * (int32_t)circBuffer[idx];
    idx = (idx == 0 ? BUFFER_SIZE - 1 : idx - 1);
  }
  bufIndex = (bufIndex + 1 < BUFFER_SIZE ? bufIndex + 1 : 0);

  acc = constrain(acc, 0, (4095LL << 15));
  uint16_t out = (uint16_t)(acc >> 15);

  DACC->DACC_CDR = out;
  while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));

  if (++dsCount >= DS_FACTOR) {
    dsCount = 0;
    if (sampleCount < SAMPLE_BUFFER_SIZE) {
      sampleBuffer[sampleCount++] = out;
      if (sampleCount >= SAMPLE_BUFFER_SIZE) {
        recordReady = true;
      }
    }
  }
}

void DACC_Handler() {
  (void)DACC->DACC_ISR;
}

void setup() {
  Serial.begin(460800);
  mymfcc.create_dct_matrix();
  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank();

  pinMode(PinTimer, OUTPUT);
  digitalWrite(PinTimer, LOW);

  setupTimer();
  setupADC();
  setupDAC();
}

void processRecording() {
  for (uint16_t offset = 0; offset + FRAME_LENGTH <= SAMPLE_BUFFER_SIZE; offset += HOP_SIZE) {
    float frame[FRAME_LENGTH];

    // 1. Conversion et centrage autour de 0, normalisation [-1, +1]
    for (uint16_t i = 0; i < FRAME_LENGTH; i++) {
      frame[i] = (sampleBuffer[offset + i] - 2048.0f) / 2048.0f;
    }

    // 2. Préaccentuation
    for (uint16_t i = FRAME_LENGTH - 1; i > 0; i--) {
      frame[i] = frame[i] - PRE_EMPHASIS_ALPHA * frame[i - 1];
    }
    frame[0] = frame[0];  // la première valeur reste comme elle est

    // 3–6. MFCC complet (fenêtre de Hamming, FFT, filtre de Mel, DCT)
    float mfcc_out[MFCC_SIZE];
    mymfcc.computeWithDCT(frame, mfcc_out);

    // Affichage
    for (uint8_t i = 0; i < MFCC_SIZE; i++) {
      Serial.print(mfcc_out[i], 4);
      if (i < MFCC_SIZE - 1)
        Serial.print(',');
      else
        Serial.println();
    }
  }

  sampleCount = 0;
  recordReady = false;
}

void loop() {
  if (recordReady) {
    processRecording();
  }
}
