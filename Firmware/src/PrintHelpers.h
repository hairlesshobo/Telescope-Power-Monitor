#include "Config.h"
#include "State.h"
#include "Helpers.h"

#include <Arduino.h>

void printWithPipe(const __FlashStringHelper *ifsh);
void printWithPipe(const String &s);
void printWithPipe(const float f);
void printWithPipe(const uint32_t ul);
void printPipePair(const __FlashStringHelper *ifsh, const String &s, boolean newline = false);

void printEnvironmentStatus(State &state, ConfigObject &config);
void printSystemStatus(State &state, ConfigObject &config);
void printAmpStatus(State &state, ConfigObject &config, float *amp_array, const __FlashStringHelper *ifsh);
void printVoltStatus(State &state, ConfigObject &config, float *volts);

void printConfigEntryHeader(const String &s);
void printConfigEntry(const String &s, int value);
void printConfigEntry(const String &s, float value);
void printConfig(ConfigObject &config);