#ifndef _HELPERS_H
#define _HELPERS_H

#include <Arduino.h>
#include "lib/RTC/RTClib.h"

void Write_P(Print &target, const char* str);
void WriteLine_P(Print &target, const char* str);
boolean StartsWith_P(const char *search, const char *str, uint8_t offset = 0);
void GetTimestamp(DateTime *dt, char *dest);

float getAvgReading(float* readingArray, uint8_t avgCount);

int getFreeMemory();

#endif // _HELPERS_H