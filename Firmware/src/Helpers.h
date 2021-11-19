#include <Arduino.h>

float getAvgReading(float* readingArray, uint8_t avgCount);
float getMinReading(float* readingArray, uint8_t avgCount);
float getMaxReading(float* readingArray, uint8_t avgCount);

int getFreeMemory();