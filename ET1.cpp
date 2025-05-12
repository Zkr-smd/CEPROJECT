// ET1 – Échantillonnage à 32 kHz sur Arduino Due avec lecture ADC et sortie DAC

#define ADC_INPUT A0 // Entrée analogique
#define DAC_OUTPUT DAC1 // Sortie analogique


volatile bool sampleReady = false;

void setup() {
  analogReadResolution(12);   // ADC: 12 bits (0-4095)
  analogWriteResolution(12);  // DAC: 12 bits (0-4095)

  // Initialisation du Timer pour 32 kHz (soit environ 31.25 µs)
  setupTimer();
}

void loop() {
  if (sampleReady) {
    sampleReady = false;

    uint16_t val = analogRead(ADC_INPUT);   // Lire le signal analogique
    analogWrite(DAC_OUTPUT, val);            // Sortir via DAC
  }
}

void setupTimer() {
  // Configuration TC0, canal 0, pour une fréquence de 32 kHz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
               TC_CMR_WAVE |         // Mode onde
               TC_CMR_WAVSEL_UP_RC | // Compteur jusqu’à RC
               TC_CMR_TCCLKS_TIMER_CLOCK1); // MCLK/2 = 42 MHz

  uint32_t rc = 21000000 / 32000 - 1; // = 655.25 -> échantillonnage à ~32kHz
  TC_SetRC(TC0, 0, rc);
  TC_Start(TC0, 0);

  // Activer l’interruption
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

// Routine d’interruption
void TC0_Handler() {
  TC_GetStatus(TC0, 0); // Clear l'interruption
  sampleReady = true;
}
