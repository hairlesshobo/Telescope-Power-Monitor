#ifndef _CONFIG_H
#define _CONFIG_H

/**
 * @brief Object used to store the configuration in the EEPROM
 */
struct ConfigObject
{
  /**
   * @brief If set to the "magic number" -1337, the EEPROM has been initialized
   */
  int Defined;

  /**
   * @brief How many readings should be averaged together
   * 
   * Valid range: 3 - 128
   */
  int AverageReadingCount;

  /**
   * @brief How many times per second voltage/current values should be read
   * 
   * Valid range: 1 - 20
   */
  float UpdateFrequency;
  
  /**
   * @brief How often (seconds) should the current status be written to serial
   * 
   * Valid range: 0.5 - 15 in 0.5 second steps
   */
  float WriteInterval;

  /**
   * @brief Fix value to add to voltage reading, can be negative
   */
  float VoltageCalibration; // 0.0585 

  /**
   * @brief Measured resistance of R1 resistor in voltage divider
   */
  int R1Actual;

  /**
   * @brief Measured resistance of R2 resistor in voltage divider
   */
  int R2Actual;

  /**
   * @brief Offset value to add to battery amperage digital readings
   * 
   * Valid range: -5 to 5 in steps of 1
   */
  int AmpDigitalOffset1;

  /**
   * @brief Offset value to add to load amperage digital readings
   * 
   * Valid range: -5 to 5 in steps of 1
   */
  int AmpDigitalOffset2;

  /**
   * @brief Offset value to add to solar amperage digital readings
   * 
   * Valid range: -5 to 5 in steps of 1
   */
  int AmpDigitalOffset3;

  /**
   * @brief Fixed value that is added to the temperate reading, can be negative
   */
  float TemperatureCalibration;

  /**
   * @brief Fixed value that is added to the humidity reading, can be negative
   */
  float HumidityCalibration;

  /**
   * @brief Humidity level, in percentage, above which the dehumidifer should be activated
   */
  int TargetHumidity;

  /**
   * @brief Humidity level, in percentage, above or below the "TargetHumidity" required to change dehumdifier state. 
   * 
   * The value provided is divided in two and added/subtracted to the "TargetHumidity" above. 
   * 
   * @example If the TargetHumidity is set to 60, and this Hysterisis value is set to 4, the dehumidifer will
   * continue running until the humidity falls below 58, and will not re-activate until after it has climbed
   * higher than 62. 
   */
  int HumidityHysterisis;
};

ConfigObject getDefaultConfig();

#endif // _CONFIG__H