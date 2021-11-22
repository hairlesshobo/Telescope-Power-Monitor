#include "PrintHelpers.h"

void PrintTimestamp(Print &target, DateTime *dtm)
{
    char buffer[20];
    GetTimestamp(dtm, buffer);

    target.print(buffer);
    target.print(F("|"));
}


void printWithPipe_P(Print &target, const char *text)
{
    Write_P(target, text);
    target.print(F("|"));
}

void printWithPipe(Print &target, const char *text)
{
    target.print(text);
    target.print(F("|"));
}

void printWithPipe(Print &target, const float f)
{
    target.print(f);
    target.print(F("|"));
}

void printWithPipe(Print &target, const int i)
{
    target.print(i);
    target.print(F("|"));
}

void printWithPipe(Print &target, const boolean b)
{
    target.print(b);
    target.print(F("|"));
}

void printWithPipe(Print &target, const uint32_t ul)
{
    target.print(ul);
    target.print(F("|"));
}

void printPipePair_P(Print &target, const char *name, const char *value, boolean newline = false)
{
    printWithPipe_P(target, name);
    target.print(value);

    if (newline)
        target.println();
}

/**
 * @brief Write temperature and humidity stats to serial
 */
void printEnvironmentStatus(Print &target, State &state, ConfigObject &config)
{
    PrintTimestamp(target, state.CurrentDtm);
    printWithPipe_P(target, STR_ENV);
    printWithPipe(target, state.Temperature);
    target.println(state.Humidity);
}

void printSystemStatus(Print &target, State &state, ConfigObject &config)
{
    PrintTimestamp(target, state.CurrentDtm);
    printWithPipe_P(target, STR_STAT);
    printWithPipe(target, state.UptimeSeconds);
    printWithPipe(target, getFreeMemory());
    printWithPipe(target, state.DehumEnabled);
    printWithPipe(target, state.TelescopeOutState);
    printWithPipe(target, state.DehumOutState);
    printWithPipe(target, state.Aux1OutState);
    printWithPipe(target, state.AcInState);
    printWithPipe(target, (uint32_t)state.BatteryCurrentStateSeconds);
    target.println(state.DehumCurrentStateSeconds);
}

void printPowerStatus(Print &target, State &state, ConfigObject &config)
{
    PrintTimestamp(target, state.CurrentDtm);
    printWithPipe_P(target, STR_PWR);

    printWithPipe(target, state.Volt);
    printWithPipe(target, state.BatteryAmp);
    printWithPipe(target, state.LoadAmp);
    printWithPipe(target, state.SolarAmp);
    target.println(state.AcAmp);
}

void printConfigEntryHeader_P(Print &target, State &state, const char *name)
{
    PrintTimestamp(target, state.CurrentDtm);
    printWithPipe_P(target, STR_CONFIG);
    printWithPipe_P(target, name);
}

void printConfigEntry_P(Print &target, State &state, const char *name, int32_t value)
{
    printConfigEntryHeader_P(target, state, name);
    target.print(value);
    target.println();
}

void printConfigEntry_P(Print &target, State &state, const char *name, uint32_t value)
{
    printConfigEntryHeader_P(target, state, name);
    target.print(value);
    target.println();
}

void printConfigEntry_P(Print &target, State &state, const char *name, float value)
{
    printConfigEntryHeader_P(target, state, name);
    target.print(value);
    target.println();
}

void printConfig(Print &target, ConfigObject &config, State &state)
{
    printConfigEntry_P(target, state, STR_AVERAGE_READING_COUNT, (uint32_t)config.AverageReadingCount);
    printConfigEntry_P(target, state, STR_UPDATE_FREQUENCY, (uint32_t)config.UpdateFrequency);
    printConfigEntry_P(target, state, STR_WRITE_INTERVAL, (uint32_t)config.WriteInterval);
    printConfigEntry_P(target, state, STR_VOLTAGE_CALIBRATION, config.VoltageCalibration);
    printConfigEntry_P(target, state, STR_R1_ACTUAL, (uint32_t)config.R1Actual);
    printConfigEntry_P(target, state, STR_R2_ACTUAL, (uint32_t)config.R2Actual);
    printConfigEntry_P(target, state, STR_AMP_DIGITAL_OFFSET_1, (int32_t)config.AmpDigitalOffset1);
    printConfigEntry_P(target, state, STR_AMP_DIGITAL_OFFSET_2, (int32_t)config.AmpDigitalOffset2);
    printConfigEntry_P(target, state, STR_AMP_DIGITAL_OFFSET_3, (int32_t)config.AmpDigitalOffset3);
    printConfigEntry_P(target, state, STR_TEMPERATURE_CALIBRATION, config.TemperatureCalibration);
    printConfigEntry_P(target, state, STR_HUMIDITY_CALIBRATION, config.HumidityCalibration);
    printConfigEntry_P(target, state, STR_TARGET_HUMIDITY, (uint32_t)config.TargetHumidity);
    printConfigEntry_P(target, state, STR_HUMIDITY_HYSTERISIS, (uint32_t)config.HumidityHysterisis);
    printConfigEntry_P(target, state, STR_AC_BACKUP_POINT, config.AcBackupPoint);
}