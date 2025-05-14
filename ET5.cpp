#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PinTimer 2
#define ADC_PIN A0
#define DAC_PIN DAC0
#define RC 656
#define TIMER_FREQUENCY 32000
#define SAMPLE_RATE 8000
#define BUFFER_SIZE 8000

#define FILTER_TAP_NUM 27
#define TAILLE_FENETRE 256
#define HOP_LENGTH int(TAILLE_FENETRE * 0.75)
#define NUM_FRAMES 41

static int16_t filter_taps[FILTER_TAP_NUM] = {
  310, 1291, 1285, 1396, 827, -123, -1148, -1701, -1313, 198,
  2580, 5191, 7215, 7977, 7215, 5191, 2580, 198, -1313, -1701,
  -1148, -123, 827, 1396, 1285, 1291, 310
};

volatile bool ready = false;
volatile int adc_value = 0;
volatile int32_t filtered_value = 0;

int16_t signal_buffer[FILTER_TAP_NUM] = {0};
int buffer_index = 0;
int downsample_counter = 0;
int downsample_factor = TIMER_FREQUENCY / SAMPLE_RATE;
bool recording = false;
int16_t audio_buffer[BUFFER_SIZE];
int sample_index = 0;

void setupTimer() {
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1;
  TC0->TC_CHANNEL[0].TC_RC = RC;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);
  digitalWrite(PinTimer, !digitalRead(PinTimer));
  adc_value = analogRead(ADC_PIN);
  ready = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Debut setup");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(PinTimer, OUTPUT);
  pinMode(ADC_PIN, INPUT);
  pinMode(DAC_PIN, OUTPUT);
  analogWriteResolution(12);

  setupTimer();
  TC_Start(TC0, 0);

  display.println("Demarrage enregistrement...");
  display.display();

  recording = true;
  sample_index = 0;
}

void loop() {
  if (ready) {
    applyFilter();

    if (downsample_counter == 0) {
      if (recording && sample_index < BUFFER_SIZE) {
        audio_buffer[sample_index++] = filtered_value >> 15;
        if (sample_index >= BUFFER_SIZE) {
          recording = false;
          Serial.println("Enregistrement terminé");
          display.clearDisplay();
          display.println("Enregistrement terminé");
          display.display();
          printFrames();
        }
      }
    }

    downsample_counter = (downsample_counter + 1) % downsample_factor;
    ready = false;
  }
}

void applyFilter() {
  signal_buffer[buffer_index] = adc_value >> 4;
  filtered_value = 0;
  int j = buffer_index;
  for (int i = 0; i < FILTER_TAP_NUM; i++) {
    filtered_value += signal_buffer[j] * filter_taps[i];
    j = (j == 0) ? FILTER_TAP_NUM - 1 : j - 1;
  }
  buffer_index = (buffer_index + 1) % FILTER_TAP_NUM;
  analogWrite(DAC_PIN, (filtered_value >> 15) & 0xFFF);
}

void printFrames() {
  Serial.println("START");
  for (int i = 0; i < NUM_FRAMES; i++) {
    for (int j = 0; j < TAILLE_FENETRE; j++) {
      int frame_index = i * HOP_LENGTH + j;
      if (frame_index < BUFFER_SIZE) {
        Serial.print(audio_buffer[frame_index]);
        if (j < TAILLE_FENETRE - 1) {
          Serial.print(", ");
        }
      }
    }
    Serial.print("FRAME ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println();
  }
  Serial.println("END");
  Serial.println("Écriture des frames terminée.");
}
