#include <Arduino.h>
#include "arduinoMFCC.h"

#define BUFFER_SIZE         45      // taps FIR
#define ADC_CH              7       // A0 → channel 7
#define DAC_CHANNEL         1       // DAC1
#define PinTimer            2       // pin pour scope
#define TIMER_RC            328     // MCK/8=10.5MHz → 10.5e6/32e3≈328
#define DS_FACTOR           4       // down-sampling 32 kHz → 8 kHz

// Enregistrement 1 s + découpage
#define SAMPLE_BUFFER_SIZE 8192     // 1 s à 8 kHz
#define FRAME_LENGTH        256
#define HOP_SIZE            (FRAME_LENGTH/2)  // 50% de recouvrement

#define MFCC_SIZE          13
#define DCT_MFCC_SIZE      13
#define FREQ_ECH           8000

// Taps du filtre FIR
static const int16_t filter_taps[BUFFER_SIZE] = {
   731,  578,  527,  248, -213, -717, -1074, -1126,  -822,  -260,
   326,  662,  548,  -33, -870, -1576, -1722, -1009,   590,  2784,
  5030, 6708, 7329, 6708, 5030, 2784,   590, -1009, -1722, -1576,
  -870,  -33,  548,  662,   326,  -260,  -822, -1126, -1074,  -717,
  -213,  248,  527,  578,   731
};

// Buffers et flags
volatile uint16_t circBuffer[BUFFER_SIZE];
volatile uint8_t  bufIndex = 0;

volatile uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
volatile uint16_t sampleCount = 0;
volatile bool     recordReady  = false;

// Objet MFCC
arduinoMFCC mymfcc(MFCC_SIZE, DCT_MFCC_SIZE, FRAME_LENGTH, FREQ_ECH);

// Variables pour down-sample
volatile uint8_t dsCount = 0;

void setupTimer() {
  PMC->PMC_PCER0 |= (1 << ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR =
       TC_CMR_WAVE
     | TC_CMR_WAVSEL_UP_RC
     | TC_CMR_TCCLKS_TIMER_CLOCK2;
  TC0->TC_CHANNEL[0].TC_RC  = TIMER_RC;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
  TC_Start(TC0, 0);
}

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR =
       ADC_MR_TRGEN_DIS
     | ADC_MR_PRESCAL(10)
     | ADC_MR_STARTUP_SUT96
     | ADC_MR_TRACKTIM(3);
  ADC->ADC_CHER = (1u << ADC_CH);
}

void setupDAC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  DACC->DACC_MR =
       DACC_MR_TRGEN_DIS
     | DACC_MR_WORD_HALF
     | DACC_MR_USER_SEL_CHANNEL1
     | DACC_MR_REFRESH(1)
     | DACC_MR_STARTUP_8
     | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH1;
  NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
  // 1) clear IRQ
  TC0->TC_CHANNEL[0].TC_SR;
  // 2) toggle pin (pour scope)
  digitalWrite(PinTimer, !digitalRead(PinTimer));
  // 3) start ADC
  ADC->ADC_CR = ADC_CR_START;
  // 4) wait end, read sample
  while ((ADC->ADC_ISR & (1u << ADC_CH)) == 0);
  uint16_t sample = ADC->ADC_CDR[ADC_CH] & 0x0FFF;
  // 5) buffer FIR
  circBuffer[bufIndex] = sample;
  // 6) calcul FIR Q15
  int32_t acc = 0;
  uint8_t idx = bufIndex;
  for (uint8_t t = 0; t < BUFFER_SIZE; t++) {
    acc += filter_taps[t] * (int32_t)circBuffer[idx];
    idx = (idx == 0 ? BUFFER_SIZE - 1 : idx - 1);
  }
  // 7) incr bufIndex
  bufIndex = (bufIndex + 1 < BUFFER_SIZE ? bufIndex + 1 : 0);
  // 8) clip & 12 bits
  acc = constrain(acc, 0, (4095LL << 15));
  uint16_t out = (uint16_t)(acc >> 15);
  // 9) write DAC
  DACC->DACC_CDR = out;
  while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));

  // 10) down-sample → buffer d’enregistrement
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

void enregistrer(uint16_t value) {
  // Récupérer les 8 bits inférieurs
  uint8_t lowByte = value & 0xFF;
  // Récupérer les 8 bits supérieurs
  uint8_t highByte = (value >> 8) & 0xFF;
  // Envoyer les deux octets via le port série
  //Serial.write(lowByte);
  //Serial.write(highByte);
}

void processRecording() {
  // Découpage en trames de 256 samples, hop=128
  //for (uint16_t i = 0; i < SAMPLE_BUFFER_SIZE; i++) {enregistrer(sampleBuffer[i]);}
  for (uint16_t offset = 0;
       offset + FRAME_LENGTH <= SAMPLE_BUFFER_SIZE;
       offset += HOP_SIZE) {

    // 1) copie locale + fenêtre
    
    float frame[FRAME_LENGTH];
    for (uint16_t i = 0; i < FRAME_LENGTH; i++) {
      frame[i] = (float)sampleBuffer[offset + i];
    }
    // 2) calcul MFCC
    float mfcc_out[MFCC_SIZE];
    mymfcc.computeWithDCT(frame, mfcc_out);
    //mymfcc.compute(frame, mfcc_out);


    // 3) affichage
    /*
    Serial.print("MFCC frame @");
    Serial.print(offset);
    Serial.print(": ");
    for (uint8_t k = 0; k < MFCC_SIZE; k++) {
      Serial.print(mfcc_out[k], 4);
      Serial.print(' ');
    }
    Serial.println();*/
    /*
    for (uint8_t i = 0; i < MFCC_SIZE; i++) {
      // Enregistrement des coefficients MFCC
      Serial.println(mfcc_out[i], 4);
    }*/

    /*
    for (uint8_t i = 0; i < MFCC_SIZE; i++) {
      int16_t val = (int16_t)(mfcc_out[i] * 100);  // mise à l’échelle
      Serial.write(val & 0xFF);
      Serial.write((val >> 8) & 0xFF);
    }
    */
    
    for (uint8_t i = 0; i < MFCC_SIZE; i++) {
      Serial.print(mfcc_out[i], 4);
      if (i < MFCC_SIZE - 1)
        Serial.print(",");
      else
        Serial.println();  // Fin de ligne = une frame MFCC
    }
    
    
  }
  // Réinitialisation pour prochain enregistrement
  sampleCount = 0;
  recordReady = false;
}

void loop() {
  if (recordReady) {
    processRecording();  // Lancer automatiquement quand buffer est plein
  }
}
