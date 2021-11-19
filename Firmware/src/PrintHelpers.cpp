#include "PrintHelpers.h"

void printWithPipe(const __FlashStringHelper *ifsh)
{
    Serial.print(ifsh);
    Serial.print(F("|"));
}

void printWithPipe(const String &s)
{
    Serial.print(s);
    Serial.print(F("|"));
}

void printWithPipe(const float f)
{
    Serial.print(f);
    Serial.print(F("|"));
}

void printWithPipe(const uint32_t ul)
{
    Serial.print(ul);
    Serial.print(F("|"));
}

void printPipePair(const __FlashStringHelper *ifsh, const String &s, boolean newline = false)
{
    printWithPipe(ifsh);

    if (newline)
        Serial.println(s);
    else
        Serial.println(s);
}

/**
 * @brief Write temperature and humidity stats to serial
 */
void printEnvironmentStatus(State &state, ConfigObject &config)
{
    printWithPipe(state.CurrentDtm->timestamp());
    printWithPipe(F("ENV"));
    printWithPipe(state.Temperature);
    Serial.println(state.Humidity);
}

void printSystemStatus(State &state, ConfigObject &config)
{
    printWithPipe(state.CurrentDtm->timestamp());
    printWithPipe(F("STAT"));
    printWithPipe(state.UptimeSeconds);
    Serial.println(getFreeMemory());
}

void printAmpStatus(State &state, ConfigObject &config, float *amp_array, const __FlashStringHelper *ifsh)
{
    printWithPipe(state.CurrentDtm->timestamp());
    printWithPipe(ifsh);
    printWithPipe(getAvgReading(amp_array, config.AverageReadingCount));
    printWithPipe(getMinReading(amp_array, config.AverageReadingCount));
    Serial.println(getMaxReading(amp_array, config.AverageReadingCount));
}

/**
 * @brief Write voltage stats to serial
 */
void printVoltStatus(State &state, ConfigObject &config, float *volts)
{
    printWithPipe(state.CurrentDtm->timestamp());
    printWithPipe(F("VOLT"));
    printWithPipe(getAvgReading(volts, config.AverageReadingCount));
    printWithPipe(getMinReading(volts, config.AverageReadingCount));
    Serial.println(getMaxReading(volts, config.AverageReadingCount));
}

void printConfigEntryHeader(const String &s)
{
    printWithPipe(F("CONFIG"));
    Serial.print(s);
    Serial.print(F(": "));
}

void printConfigEntry(const String &s, int value)
{
    printConfigEntryHeader(s);
    Serial.println(value);
}

void printConfigEntry(const String &s, float value)
{
    printConfigEntryHeader(s);
    Serial.println(value);
}

void printConfig(ConfigObject &config)
{
    printConfigEntry(F("AverageReadingCount"), config.AverageReadingCount);
    printConfigEntry(F("UpdateFrequency"), config.UpdateFrequency);
    printConfigEntry(F("WriteInterval"), config.WriteInterval);
    printConfigEntry(F("VoltageCalibration"), config.VoltageCalibration);
    printConfigEntry(F("R1Actual"), config.R1Actual);
    printConfigEntry(F("R2Actual"), config.R2Actual);
    printConfigEntry(F("AmpDigitalOffset1"), config.AmpDigitalOffset1);
    printConfigEntry(F("AmpDigitalOffset2"), config.AmpDigitalOffset2);
    printConfigEntry(F("AmpDigitalOffset3"), config.AmpDigitalOffset3);
    printConfigEntry(F("TemperatureCalibration"), config.TemperatureCalibration);
    printConfigEntry(F("HumidityCalibration"), config.HumidityCalibration);
}