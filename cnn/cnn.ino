#include <Arduino.h>
#include "weights_arduino.h" // Fichier généré par votre script Python

#ifndef FLT_MAX
#define FLT_MAX 3.4028235E+38F
#endif

#include <Arduino.h>

#define LED_BLEU_PIN 4    // LED pour la classe "BLEU"
#define LED_ROUGE_PIN 5   // LED pour la classe "ROUGE"
#define LED_NEUTRE_PIN 6  // LED neutre (optionnelle)

#define BUTTON_PIN          3   // Pin du bouton
#define BUFFER_SIZE         45
#define ADC_CH              7
#define DAC_CHANNEL         1
#define PinTimer            2
#define TIMER_RC            328
#define DS_FACTOR           4
#define SAMPLE_BUFFER_SIZE 8000   // ~1 seconde à 8kHz
#define FRAME_LENGTH        256
#define HOP_SIZE           (FRAME_LENGTH / 2)
#define MFCC_SIZE          13
#define FREQ_ECH           8000
#define PRE_EMPHASIS_ALPHA 0.97f

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
volatile bool recordReady = false;
volatile bool isRecording = false;
volatile uint8_t dsCount = 0;

float frameBuffer[FRAME_LENGTH];
float mfccBuffer[MFCC_SIZE];
float hammingWindow[FRAME_LENGTH];

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

  if (!isRecording) return;  // Ne rien faire si on n'enregistre pas

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
        isRecording = false;  // Stop recording automatiquement après 1 seconde
      }
    }
  }
}

void DACC_Handler() {
  (void)DACC->DACC_ISR;
}

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

void fft(float* x, float* y, uint16_t n) {
  uint16_t i, j, k, m;
  float wr, wi, tr, ti;

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

void computeMFCC(float* frame, float* mfcc) {
  float imag[FRAME_LENGTH] = {0};
  
  fft(frame, imag, FRAME_LENGTH);

  uint16_t spectrumSize = FRAME_LENGTH/2;
  float spectrum[spectrumSize];
  for (int i = 0; i < spectrumSize; i++) {
    spectrum[i] = frame[i]*frame[i] + imag[i]*imag[i];
  }

  float melFilters[MFCC_SIZE] = {0};
  for (int i = 0; i < MFCC_SIZE; i++) {
    int start = i * 10;
    int end = start + 20;
    if (end > spectrumSize) end = spectrumSize;
    for (int j = start; j < end; j++) {
      melFilters[i] += spectrum[j];
    }
  }

  for (int i = 0; i < MFCC_SIZE; i++) {
    if (melFilters[i] > 0) {
      melFilters[i] = log(melFilters[i]);
    } else {
      melFilters[i] = -20.0f;
    }
  }

  for (int i = 0; i < MFCC_SIZE; i++) {
    mfcc[i] = 0;
    for (int j = 0; j < MFCC_SIZE; j++) {
      mfcc[i] += melFilters[j] * cos(PI * i * (j + 0.5) / MFCC_SIZE);
    }
  }
}

void setup() {
  Serial.begin(460800);

  // Initialisation des pins LED
  pinMode(LED_BLEU_PIN, OUTPUT);
  pinMode(LED_ROUGE_PIN, OUTPUT);
  pinMode(LED_NEUTRE_PIN, OUTPUT);
  
  // Éteindre toutes les LEDs au démarrage
  digitalWrite(LED_BLEU_PIN, LOW);
  digitalWrite(LED_ROUGE_PIN, LOW);
  digitalWrite(LED_NEUTRE_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PinTimer, OUTPUT);

  setupTimer();
  setupADC();
  setupDAC();

  createHammingWindow();
}


// Ajoutez ces nouvelles fonctions pour implémenter le CNN
void conv1D(const float* input, const float* weights, const float* bias, 
            float* output, int input_len, int kernel_size, int filters) {
    int output_len = input_len - kernel_size + 1;
    for (int f = 0; f < filters; f++) {
        for (int i = 0; i < output_len; i++) {
            float sum = bias[f];
            for (int k = 0; k < kernel_size; k++) {
                sum += input[i + k] * weights[f * kernel_size + k];
            }
            output[f * output_len + i] = max(0.0f, sum); // ReLU
        }
    }
}

void maxPooling1D(const float* input, float* output, 
                 int input_len, int pool_size, int channels) {
    int output_len = input_len / pool_size;
    for (int c = 0; c < channels; c++) {
        for (int i = 0; i < output_len; i++) {
            float max_val = -FLT_MAX; // Utilisation de FLT_MAX maintenant défini
            for (int p = 0; p < pool_size; p++) {
                float val = input[c * input_len + i * pool_size + p];
                if (val > max_val) max_val = val;
            }
            output[c * output_len + i] = max_val;
        }
    }
}

void denseLayer(const float* input, const float* weights, const float* bias, 
               float* output, int input_size, int output_size) {
    for (int i = 0; i < output_size; i++) {
        float sum = bias[i];
        for (int j = 0; j < input_size; j++) {
            sum += input[j] * weights[i * input_size + j];
        }
        if (output_size > 1) { // Couche softmax (on appliquera softmax après)
            output[i] = sum;
        } else { // Couche ReLU
            output[i] = max(0.0f, sum);
        }
    }
}

void softmax(float* x, int size) {
    float max_val = x[0];
    for (int i = 1; i < size; i++) {
        if (x[i] > max_val) max_val = x[i];
    }
    
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        x[i] = exp(x[i] - max_val);
        sum += x[i];
    }
    
    for (int i = 0; i < size; i++) {
        x[i] /= sum;
    }
}

void predictCNN(float* mfcc, float* output) {
    // Couche Conv1D 1
    float conv1_out[32 * (13 - 3 + 1)]; // 32 filtres, kernel_size=3
    conv1D(mfcc, weights_0, weights_1, conv1_out, 13, 3, 32);
    
    // MaxPooling1D 1
    float pool1_out[32 * (11 / 2)]; // pool_size=2
    maxPooling1D(conv1_out, pool1_out, 11, 2, 32);
    
    // Couche Conv1D 2
    float conv2_out[64 * (5 - 3 + 1)]; // 64 filtres, kernel_size=3
    conv1D(pool1_out, weights_2, weights_3, conv2_out, 5, 3, 64);
    
    // MaxPooling1D 2
    float pool2_out[64 * (3 / 2)]; // pool_size=2
    maxPooling1D(conv2_out, pool2_out, 3, 2, 64);
    
    // Flatten
    float flatten[96]; // 64 * 1 (car 3/2=1.5 arrondi à 1)
    
    // Couche Dense 1
    float dense1_out[64];
    denseLayer(pool2_out, weights_4, weights_5, dense1_out, 96, 64);
    
    // Couche Dense 2 (sortie)
    float dense2_out[2];
    denseLayer(dense1_out, weights_6, weights_7, dense2_out, 64, 2);
    
    // Softmax
    softmax(dense2_out, 2);
    
    // Copie le résultat
    output[0] = dense2_out[0];
    output[1] = dense2_out[1];
}

// Modifiez la fonction loop pour inclure la prédiction
void loop() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && buttonState == LOW && !isRecording) {
    sampleCount = 0;
    recordReady = false;
    isRecording = true;
    
    // Éteindre les LEDs pendant l'enregistrement
    digitalWrite(LED_BLEU_PIN, LOW);
    digitalWrite(LED_ROUGE_PIN, LOW);
    digitalWrite(LED_NEUTRE_PIN, HIGH); // LED neutre allumée pendant l'enregistrement
  }

  lastButtonState = buttonState;

  if (recordReady) {
    // Éteindre la LED neutre
    digitalWrite(LED_NEUTRE_PIN, LOW);
    
    // Préparation de la trame pour MFCC
    for (int i = 0; i < FRAME_LENGTH; i++) {
      frameBuffer[i] = (float)sampleBuffer[i];
    }

    removeDCOffset(frameBuffer, FRAME_LENGTH);
    normalizeFrame(frameBuffer, FRAME_LENGTH);
    applyPreEmphasis(frameBuffer, FRAME_LENGTH);

    for (int i = 0; i < FRAME_LENGTH; i++) {
      frameBuffer[i] *= hammingWindow[i];
    }

    computeMFCC(frameBuffer, mfccBuffer);

    // Prédiction avec le CNN
    float prediction[2];
    predictCNN(mfccBuffer, prediction);
    
    // Affichage des résultats
    Serial.print("MFCC: ");
    for (int i = 0; i < MFCC_SIZE; i++) {
      Serial.print(mfccBuffer[i], 6);
      Serial.print(" ");
    }
    
    Serial.print(" | Prediction: ");
    Serial.print("Bleu=");
    Serial.print(prediction[0], 4);
    Serial.print(" Rouge=");
    Serial.print(prediction[1], 4);
    
    // Contrôle des LEDs en fonction de la prédiction
    if (prediction[0] > prediction[1]) {
      digitalWrite(LED_BLEU_PIN, HIGH);
      digitalWrite(LED_ROUGE_PIN, LOW);
      Serial.println(" | Mot détecté: BLEU");
    } else {
      digitalWrite(LED_BLEU_PIN, LOW);
      digitalWrite(LED_ROUGE_PIN, HIGH);
      Serial.println(" | Mot détecté: ROUGE");
    }

    recordReady = false;
  }
}