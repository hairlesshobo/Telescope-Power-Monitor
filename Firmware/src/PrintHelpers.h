#include "Config.h"
#include "State.h"
#include "Helpers.h"

#include <Arduino.h>

void printWithPipe(const __FlashStringHelper *ifsh);
void printWithPipe(const String &s);
void printWithPipe(const float f);
void printWithPipe(const uint32_t ul);
void printWithPipe(const boolean b);
void printWithPipe(const int i);
void printPipePair(const __FlashStringHelper *ifsh, const String &s, boolean newline = false);

void printEnvironmentStatus(State &state, ConfigObject &config);
void printSystemStatus(State &state, ConfigObject &config);
void printPowerStatus(State &state, ConfigObject &config);

void printConfigEntryHeader(const String &s);
void printConfigEntry(const String &s, int value);
void printConfigEntry(const String &s, float value);
void printConfigEntry(const String &s, uint32_t value);
void printConfig(ConfigObject &config);