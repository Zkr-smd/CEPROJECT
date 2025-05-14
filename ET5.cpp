#include <Arduino.h>
#include <SPI.h>
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

#define FRAME_SIZE 256
#define HOP_LENGTH (FRAME_SIZE / 2)  // 50% overlap = 128
#define NUM_FRAMES ((BUFFER_SIZE - FRAME_SIZE) / HOP_LENGTH + 1)

#define FILTER_TAP_NUM 21
static const int16_t filter_taps[FILTER_TAP_NUM] = {
  423, -1014, -2201, -3366, -4398, -5073, -5200, -4760,
  -3702, -2152, -324, 1655, 3612, 5351, 6615, 7180,
  6870, 5584, 3325, 236, -3345
};

// === Function Prototype ===
void printRawFrames();

volatile bool ready = false;
volatile int adc_value = 0;
volatile int32_t filtered_value = 0;

int16_t signal_buffer[FILTER_TAP_NUM] = { 0 };
int buffer_index = 0;
int downsample_counter = 0;
int downsample_factor = TIMER_FREQUENCY / SAMPLE_RATE;

bool recording = false;
int16_t audio_buffer[BUFFER_SIZE];
int sample_index = 0;

unsigned long start_time = 0;
unsigned long end_time = 0;

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
  delay(2000);  // Wait for Serial monitor or PuTTY

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(PinTimer, OUTPUT);
  pinMode(ADC_PIN, INPUT);
  pinMode(DAC_PIN, OUTPUT);
  analogWriteResolution(12);

  setupTimer();
  TC_Start(TC0, 0);

  Serial.println("Init terminee");
  display.println("Init terminee");
  display.display();
}

void loop() {
  if (!recording) {
    Serial.println("Auto-start recording");
    recording = true;
    sample_index = 0;
    start_time = millis();
  }

  if (ready) {
    applyFilter();

    if (downsample_counter == 0) {
      if (recording && sample_index < BUFFER_SIZE) {
        audio_buffer[sample_index++] = filtered_value >> 4;

        if (sample_index >= BUFFER_SIZE) {
          recording = false;
          end_time = millis();
          unsigned long recording_duration = (end_time - start_time) / 1000.0;

          Serial.print("Recording duration: ");
          Serial.print(recording_duration);
          Serial.println(" seconds");

          display.clearDisplay();
          display.setCursor(0, 0);
          display.print("Recorded in ");
          display.print(recording_duration);
          display.println(" s");
          display.display();

          delay(1000);
          printRawFrames();

          Serial.println("Done. Arduino halted.");
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("FIN");
          display.display();

          while (true);  // Freeze the Arduino
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

// ✅ Print raw 256-sample frames
void printRawFrames() {
  Serial.println("=== START RAW FRAMES ===");
  for (int i = 0; i < NUM_FRAMES; i++) {
    Serial.print("Frame ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < FRAME_SIZE; j++) {
      int frame_index = i * HOP_LENGTH + j;
      if (frame_index < BUFFER_SIZE) {
        Serial.print(audio_buffer[frame_index]);
      } else {
        Serial.print(0);
      }
      if (j < FRAME_SIZE - 1) Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println("=== END RAW FRAMES ===");
}
