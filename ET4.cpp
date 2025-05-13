#define BUFFER_SIZE 67
#define ADC_INPUT A0
#define RC 1312
#define PinTimer 13

// === Coefficients FIR ===
const int16_t filter_taps[BUFFER_SIZE] = {
  195, 550, 674, 919, 900, 740, 382, -26, -371, -513, -410, -108, 253, 500, 503, 244, -169, -540, -669,
  -457, 34, 590, 927, 823, 237, -633, -1394, -1600, -929, 661, 2878, 5167, 6883, 7519, 6883, 5167, 2878,
  661, -929, -1600, -1394, -633, 237, 823, 927, 590, 34, -457, -669, -540, -169, 244, 503, 500, 253,
  -108, -410, -513, -371, -26, 382, 740, 900, 919, 674, 550, 195
};

int tempoBuffer[BUFFER_SIZE] = {0};  // Buffer circulaire
int adcBufferIndex = 0;
int decimationCounter = 0;

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_TRGEN_DIS |
                ADC_MR_LOWRES_BITS_12 |
                ADC_MR_FREERUN_OFF |
                ADC_MR_PRESCAL(3) |
                ADC_MR_STARTUP_SUT64 |
                ADC_MR_TRANSFER(16) |
                ADC_MR_TRACKTIM(0);
  ADC->ADC_CHER = ADC_CHER_CH7;  // A0 = canal 7
}

void setupTimer() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE |
                              TC_CMR_TCCLKS_TIMER_CLOCK1 |
                              TC_CMR_WAVSEL_UP_RC;
  TC0->TC_CHANNEL[0].TC_RC = RC;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void CircularBuffer() {
  // Lire nouvel échantillon ADC
  while ((ADC->ADC_ISR & ADC_ISR_EOC7) == 0);
  tempoBuffer[adcBufferIndex] = ADC->ADC_CDR[7];

  // Met à jour l'index du buffer circulaire
  adcBufferIndex++;
  uint16_t sumIndex = adcBufferIndex;
  if (adcBufferIndex >= BUFFER_SIZE) adcBufferIndex = 0;

  // Appliquer le filtre FIR
  int32_t acc = 0;
  for (int l = 0; l < BUFFER_SIZE; l++) {
    if (sumIndex > 0) {
      sumIndex--;
    } else {
      sumIndex = BUFFER_SIZE - 1;
    }
    acc += (int32_t)filter_taps[l] * tempoBuffer[sumIndex];
  }

  // Décimation /4
  decimationCounter++;
  if (decimationCounter >= 4) {
    decimationCounter = 0;

    // Mise à l’échelle
    int32_t filtered = acc >> 15;
    if (filtered < 0) filtered = 0;
    if (filtered > 4095) filtered = 4095;

    uint16_t value = (uint16_t)filtered;

    // Envoi série (LSB puis MSB)
    uint8_t lowByte = value & 0xFF;
    uint8_t highByte = (value >> 8) & 0xFF;
    Serial.write(lowByte);
    Serial.write(highByte);
  }
}

void TC0_Handler() {
  volatile uint32_t status = TC0->TC_CHANNEL[0].TC_SR;

  digitalWrite(PinTimer, !digitalRead(PinTimer));  // Toggle pin
  ADC->ADC_CR = ADC_CR_START;                      // Démarre conversion

  CircularBuffer();  // Filtrage + décimation + transmission
}

void setup() {
  Serial.begin(460800);
  pinMode(PinTimer, OUTPUT);
  setupADC();
  setupTimer();

  // === Démarrage du timer ===
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}


void loop() {
  // Rien ici : tout est géré dans l'interruption
}
