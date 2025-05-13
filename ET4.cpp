#define BUFFER_SIZE 51
#define ADC_INPUT A0
#define RC 1312
#define PinTimer 13

// === Coefficients FIR Q15 ===
const int16_t filter_taps[BUFFER_SIZE] = {
  237, -661, -948, -1199, -1153, -748, -121, 449, 679, 452,
  -93, -621, -777, -410, 310, 951, 1049, 406, -721, -1683,
  -1737, -408, 2256, 5442, 8017, 9002, 8017, 5442, 2256, -408,
  -1737, -1683, -721, 406, 1049, 951, 310, -410, -777, -621,
  -93, 452, 679, 449, -121, -748, -1153, -1199, -948, -661, 237
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
