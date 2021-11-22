#ifndef _PRINT_HELPERS_H
#define _PRINT_HELPERS_H

#include "Config.h"
#include "State.h"
#include "Helpers.h"
#include "Strings.h"

#include <Arduino.h>

void PrintTimestamp(Print &target, DateTime *dtm);

void printWithPipe_P(Print &target, const char *text);

void printWithPipe(Print &target, const float f);
void printWithPipe(Print &target, const int i);
void printWithPipe(Print &target, const boolean b);
void printWithPipe(Print &target, const uint32_t ul);
void printWithPipe(Print &target, const char *text);

void printPipePair_P(Print &target, const char *name, const char *value, boolean newline = false);

void printEnvironmentStatus(Print &target, State &state, ConfigObject &config);
void printSystemStatus(Print &target, State &state, ConfigObject &config);
void printPowerStatus(Print &target, State &state, ConfigObject &config);

void printConfigEntry_P(Print &target, State &state, const char *name, int32_t value);
void printConfigEntry_P(Print &target, State &state, const char *name, uint32_t value);
void printConfigEntry_P(Print &target, State &state, const char *name, float value);

void printConfig(Print &target, ConfigObject &config, State &state);


#endif // _PRINT_HELPERS_H