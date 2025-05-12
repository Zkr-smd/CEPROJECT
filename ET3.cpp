const int micPin = A0; // Microphone branché sur A0
const float attenuation = 0.0316; // -30 dB en amplitude
const int sampleRate32kHz = 32000; // 32 kHz pour l'échantillonnage de l'ADC
const int sampleRate8kHz = 8000;   // 8 kHz pour l'échantillonnage final
unsigned long lastSampleMicros32kHz = 0;
unsigned long lastSampleMicros8kHz = 0;
const int intervalMicros32kHz = 1000000 / sampleRate32kHz;
const int intervalMicros8kHz = 1000000 / sampleRate8kHz;

const int bufferSize = 1024; // Taille du buffer circulaire
int buffer[bufferSize];      // Buffer circulaire pour les échantillons
int bufferIndex = 0;         // Indice courant dans le buffer

// Paramètres du filtre RIF (passe-bas à 4 kHz avec une fréquence d'échantillonnage de 32 kHz)
float cutoffFrequency = 4000.0; // Fréquence de coupure en Hz
float sampleRate = 32000.0;     // Fréquence d'échantillonnage
float RC = 1.0 / (2 * 3.1416 * cutoffFrequency);
float dt = 1.0 / sampleRate;
float alpha = dt / (RC + dt);
float previousFiltered = 0.0;

void setup() {
  analogReadResolution(12); // 12 bits de résolution
  Serial.begin(115200);
}

void loop() {
  unsigned long now = micros();

  // Échantillonnage à 32 kHz
  if (now - lastSampleMicros32kHz >= intervalMicros32kHz) {
    lastSampleMicros32kHz = now;
    int raw = analogRead(micPin);
    float centered = raw - 2048.0; // Centrage autour de 0
    float attenuated = centered * attenuation;

    // Filtrage RIF avec un filtre passe-bas (élimination des fréquences supérieures à 4 kHz)
    float filtered = previousFiltered + alpha * (attenuated - previousFiltered);
    previousFiltered = filtered;

    // Sauvegarde dans le buffer circulaire
    buffer[bufferIndex] = (int)(filtered + 2048); // Recentrer et stocker dans le buffer
    bufferIndex = (bufferIndex + 1) % bufferSize; // Gestion du buffer circulaire
  }

  // Échantillonnage à 8 kHz pour la sortie
  if (now - lastSampleMicros8kHz >= intervalMicros8kHz) {
    lastSampleMicros8kHz = now;

    // Envoi d'un échantillon du buffer circulaire via le port série
    int output = buffer[bufferIndex];
    byte highByte = (output >> 8) & 0x0F;
    byte lowByte = output & 0xFF;
    Serial.write(highByte); 
    Serial.write(lowByte);
  }
}
