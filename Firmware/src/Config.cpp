#include "Config.h"

/**
 * @brief Default values to be used when EEPROM is first initialized
 */
ConfigObject getDefaultConfig()
{
    return ConfigObject {
        -1337, // Defined
        15,    // AverageReadingCount
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
        12.1   // AcBackupPoint
    };
}