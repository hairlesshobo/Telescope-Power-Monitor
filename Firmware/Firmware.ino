/**
 *  Telescope Power Monitor - Firmware
 * 
 *  Copyright (c) 2016-2021 Steve Cross <flip@foxhollow.cc>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

// Output Format:
// <UptimeSeconds>|<AvgVoltage>|<MinVoltage>|<MaxVoltage>|<AvgAmperage>|<MinAmperage>|<MaxAmperage>|<CurrentTemperatureC>|<CurrentHumidityPercent>

// Config Options:
// AverageReadingCount => AvgRdCt
// UpdateFrequency => UdFreq
// WriteInterval => WrtInt
// VoltageCalibration => VCal
// R1Actual => R1Val
// R2Actual => R2Val
// AmpDigitalOffset => ADO
// TemperatureCalibration => TCal
// HumidityCalibration => HCal

#include "SimpleDHT.h"
// #include "RTClib.h"

#include <EEPROM.h>
// #include <SPI.h>
// #include <SD.h>

#define ENV_WRITE_DELAY 2500

const String VERSION = "2.0";
const int PIN_VOLTAGE = A0;
const int PIN_BATT_AMPERAGE = A1;
// const int PIN_
const int PIN_DHT = 7;
int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
int ACSoffset = 2500;


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
   * @brief Offset value to add to amperage digital readings
   * 
   * Valid range: -5 to 5 in steps of 1
   */
  int AmpDigitalOffset;

  /**
   * @brief Fixed value that is added to the temperate reading, can be negative
   */
  float TemperatureCalibration;

  /**
   * @brief Fixed value that is added to the humidity reading, can be negative
   */
  float HumidityCalibration;

} 

/**
 * @brief Default values to be used when EEPROM is first initialized
 */
defaultConfig = {
  -1337, // Defined
  15,    // AverageReadingCount
  4,     // UpdateFrequency
  1.0,   // WriteInterval
  0.0,   // VoltageCalibration
  10000, // R1Actual
  22000, // R2Actual
  0,     // AmpDigitalOffset
  0.0,   // TemperatureCalibration
  0.0    // HumidityCalibration
};



/**
 * @brief Non-exact number of seconds the telescope power monitor has been powered up
 * 
 * This relies on the internal clock of the ATMega chip, therfore is inexact. 
 * Additionally, this will overflow after roughly 68 years ;)
 */
long uptimeSeconds;

/**
 * @brief Array holding all readings of amperage to be averaged
 */
float* amps = 0;

/**
 * @brief Array holding all readings of voltage to be averaged
 */
float* volts = 0;

/**
 * @brief Most recent temperature reading
 */
float temperature = 0;

/**
 * @brief Most recent humidity reading
 */
float humidity = 0;

/** 
 * @brief Milliseconds of last time status was written to serial port
 */
long lastWriteTime = 0;

/**
 * @brief Milliseconds of last time DHT (temp/hum) sensor was probed
 */
long lastDhtReadTime = 0;

/**
 * @brief String object used to hold input read from serial port
 */
String inputString = "";

/**
 * @brief If true, the string has finished being read from the serial port
 */
bool stringComplete = false;

/**
 * @brief If true, the firmware will read from all sensors on board
 */
bool enableReadings = true;

/**
 * @brief Object used to hold current configuration
 */
ConfigObject config;

/**
 * @brief Instance of DHT sensor object
 */
SimpleDHT22 dht(PIN_DHT);

// Sd2Card card;
// SdVolume volume;
// SdFile root;
// RTC_PCF8523 rtc;

/**
 * @brief Initial board setup
 */
void setup() 
{
  Serial.begin(115200);  
  
  ReadConfig();

  // check for magic number
  if (config.Defined != -1337)
  {
    WriteDefaultConfig();

    ReadConfig();
  }

  uptimeSeconds = 0;

  allocateArrays();
}

/**
 * @brief Main loop
 */
void loop() 
{
  // print the string when a newline arrives:
  if (stringComplete) 
  {
    inputString.replace("\r", "");
    inputString.replace("\n", "");

    handleSerialCommand(&inputString);
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  if (enableReadings)
  {
    pushReading(volts, getVoltage(PIN_VOLTAGE));
    pushReading(amps, getAmperage(PIN_BATT_AMPERAGE));

    // pushReading(volts, 0.0);
    // pushReading(amps, 0.0);

    if (lastDhtReadTime == 0 || (millis() - lastDhtReadTime) >= ENV_WRITE_DELAY)
    {
      readDht(PIN_DHT);
      lastDhtReadTime = millis();

      writeEnvironmentStatus();
    }
  
    // we only want to send the current values periodically, even though we refresh internally multiple times per second
    if (((millis() - lastWriteTime) >= (config.WriteInterval * 1000)) || lastWriteTime == 0)
      writeStatus();
    
    delay(1000 / config.UpdateFrequency);
  }
}

/**
 * @brief Called once during board startup to allocate space for reading arrays
 */
void allocateArrays()
{
  amps = new float[config.AverageReadingCount];
  volts = new float[config.AverageReadingCount];

  initializeArray(amps);
  initializeArray(volts); 
}

/**
 * @brief Initializse the specified array with all empty values
 */
void initializeArray(float* readingArray)
{  
  for (int i = 0; i < config.AverageReadingCount; i++)
    readingArray[i] = 0.0;
}

/**
 * @brief To be called whenever the "readingsToAverage" config value is changed
 */
void reallocateArrays()
{
  if (amps != 0)
    delete [] amps;

  if (volts != 0)
    delete [] volts;

  allocateArrays();
}

/**
 * @brief Read the config from the EEPROM
 */
void ReadConfig()
{
  // load the config from the EEPROM
  EEPROM.get(0, config);
}

/**
 * @brief Write the current config object to the EEPROM
 */
void WriteConfig()
{
  // Write the current config to the EEPROM
  EEPROM.put(0, config);
}

/**
 * @brief Writes the default config values to the EEPROM
 */
void WriteDefaultConfig()
{
  Serial.println(F("Writing default config to EEPROM"));
  EEPROM.put(0, defaultConfig);
}

/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() 
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

void printPipePair(const __FlashStringHelper *ifsh, const String &s)
{
  Serial.print(ifsh);
  Serial.print(F("|"));
  Serial.print(s);
}

void printConfigEntryHeader(const String &s)
{
  Serial.print(F("CONFIG|"));
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

void handleSerialCommand(String* commandLine)
{
  if (commandLine->startsWith(F("GET")))
  {
    // strip the GET from the beginning
    *commandLine = commandLine->substring(4);

    if (commandLine->startsWith(F("Version")))
    {
      Serial.print(F("VERSION: "));
      Serial.println(VERSION);
    }
    
    else if (commandLine->startsWith(F("Config")))
    {
      printConfigEntry(F("AverageReadingCount"), config.AverageReadingCount);
      printConfigEntry(F("UpdateFrequency"), config.UpdateFrequency);
      printConfigEntry(F("WriteInterval"), config.WriteInterval);
      printConfigEntry(F("VoltageCalibration"), config.VoltageCalibration);
      printConfigEntry(F("R1Actual"), config.R1Actual);
      printConfigEntry(F("R2Actual"), config.R2Actual);
      printConfigEntry(F("AmpDigitalOffset"), config.AmpDigitalOffset);
      printConfigEntry(F("TemperatureCalibration"), config.TemperatureCalibration);
      printConfigEntry(F("HumidityCalibration"), config.HumidityCalibration);
    }
  }

  
  else if (commandLine->startsWith(F("SET")))
  {
    *commandLine = commandLine->substring(4);

    if (commandLine->startsWith(F("AverageReadingCount")))
    {
      *commandLine = commandLine->substring(20);
      config.AverageReadingCount = commandLine->toInt();
      
      printConfigEntry(F("AverageReadingCount"), config.AverageReadingCount);

      reallocateArrays();
    }

    else if (commandLine->startsWith(F("UpdateFrequency")))
    {
      *commandLine = commandLine->substring(16);
      config.UpdateFrequency = commandLine->toFloat();
      
      printConfigEntry(F("UpdateFrequency"), config.UpdateFrequency);
    }
    
    else if (commandLine->startsWith(F("WriteInterval")))
    {
      *commandLine = commandLine->substring(14);
      config.WriteInterval = commandLine->toFloat();
      
      printConfigEntry(F("WriteInterval"), config.WriteInterval);
    }
    
    else if (commandLine->startsWith(F("VoltageCalibration")))
    {
      *commandLine = commandLine->substring(19);
      config.VoltageCalibration = commandLine->toFloat();
      
      printConfigEntry(F("VoltageCalibration"), config.VoltageCalibration);
    }
    
    else if (commandLine->startsWith(F("R1Actual")))
    {
      *commandLine = commandLine->substring(9);
      config.R1Actual = commandLine->toInt();
      
      printConfigEntry(F("R1Actual"), config.R1Actual);
    }

    else if (commandLine->startsWith(F("R2Actual")))
    {
      *commandLine = commandLine->substring(9);
      config.R2Actual = commandLine->toInt();
      
      printConfigEntry(F("R2Actual"), config.R2Actual);
    }

    else if (commandLine->startsWith(F("AmpDigitalOffset")))
    {
      *commandLine = commandLine->substring(17);
      config.AmpDigitalOffset = commandLine->toInt();
      
      printConfigEntry(F("AmpDigitalOffset"), config.AmpDigitalOffset);
    }

    else if (commandLine->startsWith(F("TemperatureCalibration")))
    {
      *commandLine = commandLine->substring(23);
      config.TemperatureCalibration = commandLine->toFloat();
      
      printConfigEntry(F("TemperatureCalibration"), config.TemperatureCalibration);
    }

    else if (commandLine->startsWith(F("HumidityCalibration")))
    {
      *commandLine = commandLine->substring(20);
      config.HumidityCalibration = commandLine->toFloat();
      
      printConfigEntry(F("HumidityCalibration"), config.HumidityCalibration);
    }
  }

  
  else if (commandLine->startsWith(F("SAVE")))
  {
    WriteConfig();
    Serial.println(F("EEPROM Written"));
  }

  
  else if (commandLine->startsWith(F("CLEAR")))
  {
    WriteDefaultConfig();
    ReadConfig();
  }

  else if (commandLine->startsWith(F("PAUSE")))
  {
    enableReadings = false;
  }

  else if (commandLine->startsWith(F("RESUME")))
  {
    enableReadings = true;
  }
}

void writeStatus()
{
  uptimeSeconds = millis() / 1000;

  Serial.print(uptimeSeconds);
  Serial.print(F("|"));
  
  Serial.print(F("BATT"));
  Serial.print(F("|"));

  // output voltage stats
  Serial.print(getAvgReading(volts));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(volts));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(volts));
  Serial.print(F("|"));
  
  // output amperage stats
  Serial.print(getAvgReading(amps));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(amps));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(amps));
  Serial.println();

  lastWriteTime = millis();
}

void writeEnvironmentStatus()
{
  uptimeSeconds = millis() / 1000;

  Serial.print(uptimeSeconds);
  Serial.print(F("|"));

  Serial.print(F("ENV"));
  Serial.print(F("|"));
    
  // output temperature and humidity stats
  Serial.print(temperature);
  Serial.print(F("|"));
  
  Serial.print(humidity);
  Serial.println();

}

float getVoltage(int pin)
{
  int voltSensorValue = analogRead(pin);

  float pinVoltage = ((float)voltSensorValue / 1024.0) * 5.0;

  float denominator = (float)config.R1Actual / (config.R1Actual + config.R2Actual);

  return ((float)pinVoltage / denominator) + config.VoltageCalibration;
}

float getAmperage(int pin)
{
  int ampSensorValue = analogRead(pin);

  // calibration logic.. rather rudimentary
  ampSensorValue += config.AmpDigitalOffset;

  if (ampSensorValue > 1024)
    ampSensorValue = 1024;

  if (ampSensorValue < 0)
    ampSensorValue = 0;

  float tmpVoltage = (ampSensorValue / 1024.0) * 5000; // Gets you mV
  return ((tmpVoltage - ACSoffset) / mVperAmp);
}

void readDht(int pin)
{
  dht.read2(&temperature, &humidity, NULL);
}

void pushReading(float* readingArray, float newValue)
{
  // work backwards through the array, moving the contents back by one element
  for (int i = config.AverageReadingCount-2; i >= 0; i--)
  {
    readingArray[i+1] = readingArray[i];
  }

  // set the first value to the new provided value
  readingArray[0] = newValue;
}

float getAvgReading(float* readingArray)
{
  int count = 0;
  float total = 0.0;

  for (int i = 0; i < config.AverageReadingCount; i++)
  {
    total += readingArray[i];
    count++;
  }

  return (float)total/count;
}

float getMinReading(float* readingArray)
{
  float lastMin = 0;
  
  for (int i = 0; i < config.AverageReadingCount; i++)
  {
    if (readingArray[i] <= lastMin || lastMin == 0)
      lastMin = readingArray[i];
  }

  return lastMin;
}

float getMaxReading(float* readingArray)
{
  float lastMax = 0.0;
  
  for (int i = 0; i < config.AverageReadingCount; i++)
  {
    if (readingArray[i] >= lastMax)
      lastMax = readingArray[i];
  }

  return lastMax;
}

