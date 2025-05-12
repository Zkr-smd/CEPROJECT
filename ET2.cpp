#define FILTER_TAP_NUM 32

float firCoeffs[FILTER_TAP_NUM] = { /* paste your real coeffs here */ };
float firBuffer[FILTER_TAP_NUM] = {0};
uint8_t bufferIndex = 0;

float applyFIRFilter(float input) {
  firBuffer[bufferIndex] = input;

  float output = 0.0;
  uint8_t index = bufferIndex;
  for (uint8_t i = 0; i < FILTER_TAP_NUM; i++) {
    output += firCoeffs[i] * firBuffer[index];
    if (index == 0)
      index = FILTER_TAP_NUM - 1;
    else
      index--;
  }

  bufferIndex++;
  if (bufferIndex >= FILTER_TAP_NUM)
    bufferIndex = 0;

  return output;
}
