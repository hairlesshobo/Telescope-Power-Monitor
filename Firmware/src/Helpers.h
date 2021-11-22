#ifndef _HELPERS_H
#define _HELPERS_H

#include <Arduino.h>
#include "lib/RTC/RTClib.h"

void write_p(Print &target, const char* str);
void writeLine_p(Print &target, const char* str);
boolean startsWith_p(const char *search, const char *str, uint8_t offset = 0);
void getTimestamp(DateTime *dt, char *dest);

float getAvgReading(float* readingArray, uint8_t avgCount);

int getFreeMemory();

#endif // _HELPERS_H