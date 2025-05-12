const int micPin = A0;      // Micro branché sur A0
const float attenuation = 0.0316; // -30 dB en amplitude
const int sampleRate = 8000; // 8 kHz (ajustable)
unsigned long lastSampleMicros = 0;
const int intervalMicros = 1000000 / sampleRate;

void setup() {
  analogReadResolution(12);
  Serial.begin(115200);
}

void loop() {
  unsigned long now = micros();
  if (now - lastSampleMicros >= intervalMicros) {
    lastSampleMicros = now;

    int raw = analogRead(micPin);
    float centered = raw - 2048.0;
    float attenuated = centered * attenuation;

    int output = (int)(attenuated + 2048);
    output = constrain(output, 0, 4095); 

    byte highByte = (output >> 8) & 0x0F;
    byte lowByte = output & 0xFF;

    Serial.write(highByte); 
    Serial.write(lowByte); 
  }
}
