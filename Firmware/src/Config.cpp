#include "Config.h"

/**
 * @brief Default values to be used when EEPROM is first initialized
 */
ConfigObject getDefaultConfig()
{
    return ConfigObject {
        -1337, // Defined
        10,    // AverageReadingCount
        2,     // UpdateFrequency
        2,     // WriteInterval
        0.0,   // VoltageCalibration
        10000, // R1Actual
        22000, // R2Actual
        0,     // AmpDigitalOffset1
        0,     // AmpDigitalOffset2
        0,     // AmpDigitalOffset3
        0.0,   // TemperatureCalibration
        0.0,   // HumidityCalibration
        60,    // TargetHumidity
        4,     // HumidityHysterisis
        50,    // AcBackupPoint
        12,    // BatteryCapacityAh
        0.3,   // BatteryEndingAmps
        14.4   // BatteryAbsorbVoltage
    };
}