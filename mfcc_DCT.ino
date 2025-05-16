#include <Arduino.h>

// Original configuration from your code
#define BUFFER_SIZE         45
#define ADC_CH              7
#define DAC_CHANNEL         1
#define PinTimer            2
#define TIMER_RC            328
#define DS_FACTOR           4
#define SAMPLE_BUFFER_SIZE 8000
#define FRAME_LENGTH        256
#define HOP_SIZE           (FRAME_LENGTH / 2)
#define MFCC_SIZE          13
#define DCT_MFCC_SIZE      13
#define FREQ_ECH           8000
#define PRE_EMPHASIS_ALPHA 0.97f

// Original filter taps from your code
static const int16_t filter_taps[BUFFER_SIZE] = {
   731,  578,  527,  248, -213, -717, -1074, -1126,  -822,  -260,
   326,  662,  548,  -33, -870, -1576, -1722, -1009,   590,  2784,
  5030, 6708, 7329, 6708, 5030, 2784,   590, -1009, -1722, -1576,
  -870,  -33,  548,  662,   326,  -260,  -822, -1126, -1074,  -717,
  -213,  248,  527,  578,   731
};

// Original variables from your code
volatile uint16_t circBuffer[BUFFER_SIZE];
volatile uint8_t bufIndex = 0;
volatile uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
volatile uint16_t sampleCount = 0;
volatile bool recordReady = false;
volatile uint8_t dsCount = 0;

// MFCC Processing buffers
float frameBuffer[FRAME_LENGTH];
float mfccBuffer[MFCC_SIZE];
float hammingWindow[FRAME_LENGTH];

// ===== Original Functions from Your Code =====

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

// ===== Improved MFCC Processing =====

void createHammingWindow() {
  for (int i = 0; i < FRAME_LENGTH; i++) {
    hammingWindow[i] = 0.54f - 0.46f * cos(2 * PI * i / (FRAME_LENGTH - 1));
  }
}

void removeDCOffset(float* frame, uint16_t length) {
  float mean = 0.0f;
  for (int i = 0; i < length; i++) {
    mean += frame[i];
  }
  mean /= length;
  
  for (int i = 0; i < length; i++) {
    frame[i] -= mean;
  }
}

void normalizeFrame(float* frame, uint16_t length) {
  float maxVal = 0.0f;
  for (int i = 0; i < length; i++) {
    if (abs(frame[i]) > maxVal) maxVal = abs(frame[i]);
  }
  
  if (maxVal > 0.001f) {
    for (int i = 0; i < length; i++) {
      frame[i] /= maxVal;
    }
  }
}

void applyPreEmphasis(float* frame, uint16_t length) {
  for (int i = length - 1; i > 0; i--) {
    frame[i] -= PRE_EMPHASIS_ALPHA * frame[i - 1];
  }
}

// FFT implementation (simplified Radix-2)
void fft(float* x, float* y, uint16_t n) {
  uint16_t i, j, k, m;
  float wr, wi, tr, ti;
  
  // Bit-reverse
  j = 0;
  for (i = 0; i < n-1; i++) {
    if (i < j) {
      tr = x[j];
      ti = y[j];
      x[j] = x[i];
      y[j] = y[i];
      x[i] = tr;
      y[i] = ti;
    }
    k = n/2;
    while (k <= j) {
      j -= k;
      k /= 2;
    }
    j += k;
  }
  
  // Butterfly
  float angle;
  for (m = 1; m < n; m *= 2) {
    for (k = 0; k < m; k++) {
      angle = -PI * k / m;
      wr = cos(angle);
      wi = sin(angle);
      for (i = k; i < n; i += 2*m) {
        j = i + m;
        tr = wr * x[j] - wi * y[j];
        ti = wr * y[j] + wi * x[j];
        x[j] = x[i] - tr;
        y[j] = y[i] - ti;
        x[i] += tr;
        y[i] += ti;
      }
    }
  }
}

// MFCC calculation with DCT
void computeMFCC(float* frame, float* mfcc) {
  float imag[FRAME_LENGTH] = {0};
  
  // 1. Compute FFT
  fft(frame, imag, FRAME_LENGTH);
  
  // 2. Compute power spectrum (first half)
  uint16_t spectrumSize = FRAME_LENGTH/2;
  float spectrum[spectrumSize];
  for (int i = 0; i < spectrumSize; i++) {
    spectrum[i] = frame[i]*frame[i] + imag[i]*imag[i];
  }
  
  // 3. Apply mel filter bank (example triangular filters)
  float melFilters[MFCC_SIZE] = {0};
  // This is a simplified example - you should implement proper mel filter banks
  for (int i = 0; i < MFCC_SIZE; i++) {
    int start = i * 10;
    int end = start + 20;
    if (end > spectrumSize) end = spectrumSize;
    for (int j = start; j < end; j++) {
      melFilters[i] += spectrum[j];
    }
  }
  
  // 4. Log compression
  for (int i = 0; i < MFCC_SIZE; i++) {
    if (melFilters[i] > 0) {
      melFilters[i] = log(melFilters[i]);
    } else {
      melFilters[i] = -20.0f; // Small value for log(0)
    }
  }
  
  // 5. DCT (Type II)
  for (int i = 0; i < MFCC_SIZE; i++) {
    mfcc[i] = 0;
    for (int j = 0; j < MFCC_SIZE; j++) {
      mfcc[i] += melFilters[j] * cos(PI * i * (j + 0.5f) / MFCC_SIZE);
    }
    // Optional: Multiply by sqrt(2/N) scale factor
    mfcc[i] *= sqrt(2.0f / MFCC_SIZE);
  }
}

void processRecording() {
  for (uint16_t offset = 0; offset + FRAME_LENGTH <= SAMPLE_BUFFER_SIZE; offset += HOP_SIZE) {
    // 1. Convert and center around 0, normalize [-1, +1]
    for (uint16_t i = 0; i < FRAME_LENGTH; i++) {
      frameBuffer[i] = (sampleBuffer[offset + i] - 2048.0f) / 2048.0f;
    }
    
    // 2. Signal conditioning
    removeDCOffset(frameBuffer, FRAME_LENGTH);
    normalizeFrame(frameBuffer, FRAME_LENGTH);
    applyPreEmphasis(frameBuffer, FRAME_LENGTH);
    
    // 3. Apply window
    for (int i = 0; i < FRAME_LENGTH; i++) {
      frameBuffer[i] *= hammingWindow[i];
    }
    
    // 4. Compute MFCCs
    computeMFCC(frameBuffer, mfccBuffer);
    
    // 5. Output results (same format as original)
    for (uint8_t i = 0; i < MFCC_SIZE; i++) {
      Serial.print(mfccBuffer[i], 4);
      if (i < MFCC_SIZE - 1)
        Serial.print(',');
      else
        Serial.println();
    }
  }
  
  sampleCount = 0;
  recordReady = false;
}

// ===== Setup & Main Loop =====

void setup() {
  Serial.begin(460800);
  createHammingWindow();
  
  pinMode(PinTimer, OUTPUT);
  digitalWrite(PinTimer, LOW);

  setupTimer();
  setupADC();
  setupDAC();
}

void loop() {
  if (recordReady) {
    processRecording();
  }
}
