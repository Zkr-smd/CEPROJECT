////////////////////////////////////////////////////////////////////
// LIBRARIES ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MFCC.h>
 // Include the MFCC library
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// CONSTANT AND GLOBAL VARIABLES ///////////////////////////////////
////////////////////////////////////////////////////////////////////
#define FRAME_SIZE 256
#define HOP_SIZE 128
#define MFCC_SIZE 13

float frames[FRAME_SIZE];
float mfccs[MFCC_SIZE];
MFCC mfcc(FRAME_SIZE);
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // (-1 partage le pin de reset Arduino)
#define SCREEN_ADDRESS 0x3C  //0x3C pourr 128x32
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//ADC
#define MAX_ADC 4095
#define MIN_ADC 0

//SAMPLING
const int samplingFrequency = 44000;

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44000 Hz

* 20 Hz - 16000 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 2.1129857238161254 dB

* 17000 Hz - 22000 Hz
  gain = 0
  desired attenuation = -10 dB
  actual attenuation = -10.97955300799253 dB

*/

#define FILTER_TAP_NUM 21

static double filter_taps_float[FILTER_TAP_NUM] = {
  -0.06072852230688862,
  -0.009743879979818206,
  0.03554309359439512,
  -0.06453910915450291,
  0.07912618274311577,
  -0.06366899994206501,
  0.012191671871374815,
  0.06710767199756262,
  -0.15331654700903166,
  0.21994704013789626,
  0.7550762235546863,
  0.21994704013789626,
  -0.15331654700903166,
  0.06710767199756262,
  0.012191671871374815,
  -0.06366899994206501,
  0.07912618274311577,
  -0.06453910915450291,
  0.03554309359439512,
  -0.009743879979818206,
  -0.06072852230688862
};

uint16_t filteredValue;

//ADC BUFFERS
int position = 0;
#define BUFFER_SIZE FILTER_TAP_NUM
volatile uint16_t adc_buffer[BUFFER_SIZE];  //doit etre sur 16bits
float filtered_buffer[BUFFER_SIZE];

bool enregistrement = false;

//PINS
// A0 - microphone
// D4 - debut enregistrement
// D5 - fin enregistrement
// D6 - led enregistrement
const int PIN_LED_ENREGISTREMENT = 6;
const int PIN_BOUTON_DEBUT = 4;
const int PIN_BOUTON_FIN = 10;

//FRAMES

uint16_t sample;  // Variable pour stocker chaque échantillon audio
// Déclarez le tableau de frames
int numFrames = 0;           // Nombre de frames extraites
int numSamplesInBuffer = 0;  // Nombre d'échantillons actuellement présents dans le tampon
int nbValeurRecouvrement = 10;
uint16_t frames[175][256];
int rang1 = 0;
int rang2;

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// SETUP ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);

  setup_ADC();
  setup_DAC();
  setup_display();

  pinMode(PIN_BOUTON_DEBUT, INPUT);
  pinMode(PIN_BOUTON_FIN, INPUT);
  pinMode(PIN_LED_ENREGISTREMENT, OUTPUT);

  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    filtered_buffer[i] = 0;
  }
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// SETUPS //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//function used to setup the OLED display
void setup_display() {
  if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  _display.clearDisplay();
  _display.setTextColor(WHITE);
  _display.setCursor(2, 2);
  _display.print(F("FS4"));
  _display.display();
}

//function used to setup the analogic / digital converter
void setup_ADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;     // Active le périphérique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(255)      // Définit le diviseur de fréquence à 255
                | ADC_MR_STARTUP_SUT64   // Définit le temps de démarrage à 64 périodes d'ADC_CLK
                | ADC_MR_TRACKTIM(15)    // Définit le temps de suivi à 15 périodes d'ADC_CLK
                | ADC_MR_SETTLING_AST3;  // Définit le temps de stabilisation à 17 périodes d'ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;          // Active le canal 7 (A0)

  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;  // Active le périphérique TC0
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Active le déclenchement de comparaison RC
  // Définit la valeur RC pour une fréquence samplingFrequency Hz
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);

  // Configure le contrôleur DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39;                  // Active le périphérique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;  // Désactive le transfert PDC
  ADC->ADC_RPR = (uint32_t)adc_buffer;                // Définit le pointeur de réception sur le tampon
  ADC->ADC_RCR = BUFFER_SIZE;                         // Définit le compteur de réception à la taille du tampon
  ADC->ADC_RNPR = (uint32_t)adc_buffer;               // Définit le prochain pointeur de réception sur le tampon
  ADC->ADC_RNCR = BUFFER_SIZE;                        // Définit le prochain compteur de réception à la taille du tampon
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;                     // Active le transfert PDC
}

void setup_DAC() {
  // Active le périphérique DAC
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  // Configure le DAC en mode normal
  DACC->DACC_MR = DACC_MR_REFRESH(1) | DACC_MR_STARTUP_8 | DACC_MR_MAXS | 0x10000;
  // Active le canal 1 du DAC
  DACC->DACC_CHER = DACC_CHER_CH1;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// HANDLERS ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//function used to handle clock TC0 interrupt
void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption
  TC0->TC_CHANNEL[0].TC_SR;
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// LOOP ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void loop() {
  // Vérifie si le transfert DMA est terminé
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Désactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    filtersignalRIF2();

    // Frame separation and MFCC calculation
    for(int i=0; i<BUFFER_SIZE; i+=HOP_SIZE) {
      for(int j=0; j<FRAME_SIZE; j++) {
        frames[j] = filtered_buffer[(i+j)%BUFFER_SIZE];
      }
      mfcc.computeMFCC(frames, mfccs);
      for(int j=0; j<MFCC_SIZE; j++) {
        Serial.println(mfccs[j]);
      }
    }
    
    // Réactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    // Réinitialise le pointeur de réception et le compteur
    ADC->ADC_RPR = (uint32_t)&adc_buffer[position];
    ADC->ADC_RCR = 1;

    // Réinitialise le prochain pointeur de réception et le compteur
    ADC->ADC_RNPR = (uint32_t)&adc_buffer[position];
    ADC->ADC_RNCR = 1;
  }
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
// FILTRAGE ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void filtersignalRIF2() {
  filteredValue = 0;  // calcul de la nouvelle valeur filtrée
  //filtrage avec tampon circulaire
  for (int l = 0; l < FILTER_TAP_NUM; l++) {
    if ((position - l) < 0) {
      filteredValue += filter_taps_float[l] * adc_buffer[position - l + FILTER_TAP_NUM];
    } else {
      filteredValue += filter_taps_float[l] * adc_buffer[position - l];
    }
  }
  position = (position + 1) % FILTER_TAP_NUM;
}
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////