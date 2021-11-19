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
// <ISO_DTM>|STAT|<UptimeSeconds>|<BytesFreeMem>
// <ISO_DTM>|VOLT|<AvgVoltage>|<MinVoltage>|<MaxVoltage>
// <ISO_DTM>|BATT|<AvgAmperage>|<MinAmperage>|<MaxAmperage>
// <ISO_DTM>|LOAD|<AvgAmperage>|<MinAmperage>|<MaxAmperage>
// <ISO_DTM>|SOLAR|<AvgAmperage>|<MinAmperage>|<MaxAmperage>
// <ISO_DTM>|AC|<AvgAmperage>|<MinAmperage>|<MaxAmperage>
// <ISO_DTM>|ENV|<CurrentTemperatureC>|<CurrentHumidityPercent>

// Config Options:
// AverageReadingCount => AvgRdCt
// UpdateFrequency => UdFreq
// WriteInterval => WrtInt
// VoltageCalibration => VCal
// R1Actual => R1Val
// R2Actual => R2Val
// AmpDigitalOffset1 => ADO1
// AmpDigitalOffset2 => ADO2
// AmpDigitalOffset3 => ADO3
// TemperatureCalibration => TCal
// HumidityCalibration => HCal

#include "ConfigObject.h"

#include "SimpleDHT.h"
#include "RTClib.h"

#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

#define ENV_WRITE_DELAY 2500


// Constants
const String VERSION = "2.0";

const uint8_t mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module

const uint16_t ACSoffset = 2500;

// Pin Definitions
const uint8_t PIN_VOLTAGE = A0;
const uint8_t PIN_BATT_AMPERAGE = A1;
const uint8_t PIN_LOAD_AMPERAGE = A2;
const uint8_t PIN_SOLAR_AMPERAGE = A3;
// const uint8_t PIN_SDA = A4;
// const uint8_t PIN_SCL = A5;

// const uint8_t PIN_RX = 0;
// const uint8_t PIN_TX = 1;
const uint8_t PIN_TOGGLE_TELESCOPE_OUTPUT_BUTTON = 2;
const uint8_t PIN_TOGGLE_DEHUMIDIFIER_OUTPUT_BUTTON = 3;
const uint8_t PIN_TELESCOPE_OUTPUT_RELAY = 4;
const uint8_t PIN_DEHUMIDIFIER_OUTPUT_RELAY = 5;
const uint8_t PIN_AUX_OUTPUT_RELAY = 6;
const uint8_t PIN_DHT = 7;
const uint8_t PIN_AC_INPUT_RELAY = 8;
const uint8_t PIN_DEHUMIDIFIER_ENABLED_LED = 9;
const uint8_t PIN_SD_SELECT = 10;
// const uint8_t PIN_ICSP_MOSI = 11;
// const uint8_t PIN_ICSP_MISO = 12;
// const uint8_t PIN_ICSP_SCK = 13;



/**
 * @brief Non-exact number of seconds the telescope power monitor has been powered up
 * 
 * This relies on the internal clock of the ATMega chip, therfore is inexact. 
 * Additionally, this will overflow after roughly 68 years ;)
 */
uint32_t uptimeSeconds;

/**
 * @brief Array holding all readings of battery amperage to be averaged
 */
float* amps_battery = 0;

/**
 * @brief Array holding all readings of load amperage to be averaged
 */
float* amps_load = 0;

/**
 * @brief Array holding all readings of solar amperage to be averaged
 */
float* amps_solar = 0;

/**
 * @brief Array holding all readings of AC amperage to be averaged
 */
float* amps_ac = 0;

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
uint32_t lastWriteTime = 0;

/**
 * @brief Last voltage/amperage sensor read time
 */
uint32_t lastReadTime = 0;

/**
 * @brief Milliseconds of last time the system clock and status was updated
 */
uint32_t lastTick = 0;

/**
 * @brief Milliseconds of last time DHT (temp/hum) sensor was probed
 */
uint32_t lastDhtReadTime = 0;

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

/**
 * @brief Most recent DTM as read from the RTC
 */
DateTime* currentDtm;

Sd2Card card;
SdVolume volume;
SdFile root;

RTC_PCF8523 rtc;

/**
 * @brief Initial board setup
 */
void setup() 
{
  inputString.reserve(30);

  Serial.begin(115200);  
  
  readConfig();

  // check for magic number
  if (config.Defined != -1337)
  {
    writeDefaultConfig();

    readConfig();
  }

  uptimeSeconds = 0;

  allocateArrays();

  pinMode(PIN_DEHUMIDIFIER_ENABLED_LED, OUTPUT);
  // digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, HIGH);

  // TODO: Remove from final code
  currentDtm = new DateTime(2021, 11, 19, 13, 27, 0);
}

// TODO: remove for final code
int lastStatus = LOW;

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

  // update the clock
  if (lastTick == 0 || (millis() - lastTick) >= 1000)
  {
    uptimeSeconds = millis() / 1000L;
    updateDtm();

    // TODO: For testing only.. remove for final code
    lastStatus = !lastStatus;
    digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, lastStatus);

    writeSystemStatus();

    lastTick = millis();
  }

  if (enableReadings)
  {
    if (lastReadTime == 0 || (millis() - lastReadTime) >= config.UpdateFrequency)
    {
      pushReading(volts, getVoltage(PIN_VOLTAGE));
      pushReading(amps_battery, getAmperage(PIN_BATT_AMPERAGE));
      pushReading(amps_load, getAmperage(PIN_LOAD_AMPERAGE));
      pushReading(amps_solar, getAmperage(PIN_SOLAR_AMPERAGE));

      // TODO: calculate AC amperage
      pushReading(amps_ac, 0.0);

      lastReadTime = millis();
    }

    if (lastDhtReadTime == 0 || (millis() - lastDhtReadTime) >= ENV_WRITE_DELAY)
    {
      readDht(PIN_DHT, temperature, humidity);
      writeEnvironmentStatus();

      lastDhtReadTime = millis();
    }
  
    // we only want to send the current values periodically, even though we refresh internally multiple times per second
    if (((millis() - lastWriteTime) >= (config.WriteInterval * 1000)) || lastWriteTime == 0)
    {
      writeVoltStatus();
      writeBattStatus();
      writeLoadStatus();
      writeSolarStatus();
      writeAcStatus();

      lastWriteTime = millis();
    }


    
    // delay(1000 / config.UpdateFrequency);
  }
}

/**
 * @brief Called once during board startup to allocate space for reading arrays
 */
void allocateArrays()
{
  volts = new float[config.AverageReadingCount];
  amps_battery = new float[config.AverageReadingCount];
  amps_load = new float[config.AverageReadingCount];
  amps_solar = new float[config.AverageReadingCount];

  initializeArray(volts); 
  initializeArray(amps_battery);
  initializeArray(amps_load);
  initializeArray(amps_solar);
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
  if (amps_battery != 0)
    delete [] amps_battery;

  if (volts != 0)
    delete [] volts;

  if (amps_load != 0)
    delete [] amps_load;

  if (amps_solar != 0)
    delete [] amps_solar;

  allocateArrays();
}

/**
 * @brief Read the config from the EEPROM
 */
void readConfig()
{
  // load the config from the EEPROM
  EEPROM.get(0, config);
}

/**
 * @brief Write the current config object to the EEPROM
 */
void writeConfig()
{
  // Write the current config to the EEPROM
  EEPROM.put(0, config);
}

/**
 * @brief Writes the default config values to the EEPROM
 */
void writeDefaultConfig()
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
      printConfigEntry(F("AmpDigitalOffset1"), config.AmpDigitalOffset1);
      printConfigEntry(F("AmpDigitalOffset2"), config.AmpDigitalOffset2);
      printConfigEntry(F("AmpDigitalOffset3"), config.AmpDigitalOffset3);
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

    else if (commandLine->startsWith(F("AmpDigitalOffset1")))
    {
      *commandLine = commandLine->substring(17);
      config.AmpDigitalOffset1 = commandLine->toInt();
      
      printConfigEntry(F("AmpDigitalOffset1"), config.AmpDigitalOffset1);
    }

    else if (commandLine->startsWith(F("AmpDigitalOffset2")))
    {
      *commandLine = commandLine->substring(17);
      config.AmpDigitalOffset2 = commandLine->toInt();
      
      printConfigEntry(F("AmpDigitalOffset2"), config.AmpDigitalOffset2);
    }

        else if (commandLine->startsWith(F("AmpDigitalOffset3")))
    {
      *commandLine = commandLine->substring(17);
      config.AmpDigitalOffset3 = commandLine->toInt();
      
      printConfigEntry(F("AmpDigitalOffset3"), config.AmpDigitalOffset3);
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
    writeConfig();
    Serial.println(F("EEPROM Written"));
  }

  
  else if (commandLine->startsWith(F("CLEAR")))
  {
    writeDefaultConfig();
    readConfig();
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

void writeVoltStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));
  
  Serial.print(F("VOLT"));
  Serial.print(F("|"));

  // output voltage stats
  Serial.print(getAvgReading(volts));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(volts));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(volts));
  Serial.println();
}

void writeBattStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));
  
  Serial.print(F("BATT"));
  Serial.print(F("|"));

  // output amperage stats
  Serial.print(getAvgReading(amps_battery));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(amps_battery));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(amps_battery));
  Serial.println();
}

void writeSolarStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));
  
  Serial.print(F("SOLAR"));
  Serial.print(F("|"));

  // output amperage stats
  Serial.print(getAvgReading(amps_solar));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(amps_solar));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(amps_solar));
  Serial.println();
}

void writeAcStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));
  
  Serial.print(F("AC"));
  Serial.print(F("|"));

  // output amperage stats
  Serial.print(getAvgReading(amps_ac));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(amps_ac));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(amps_ac));
  Serial.println();
}

void writeLoadStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));
  
  Serial.print(F("LOAD"));
  Serial.print(F("|"));

  // output amperage stats
  Serial.print(getAvgReading(amps_load));
  Serial.print(F("|"));
  
  Serial.print(getMinReading(amps_load));
  Serial.print(F("|"));
  
  Serial.print(getMaxReading(amps_load));
  Serial.println();
}

void writeEnvironmentStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));

  Serial.print(F("ENV"));
  Serial.print(F("|"));
    
  // output temperature and humidity stats
  Serial.print(temperature);
  Serial.print(F("|"));
  
  Serial.print(humidity);
  Serial.println();
}

void writeSystemStatus()
{
  Serial.print(currentDtm->timestamp());
  Serial.print(F("|"));

  Serial.print(F("STAT"));
  Serial.print(F("|"));

  Serial.print(uptimeSeconds);
  Serial.print(F("|"));

  Serial.print(getFreeMemory());
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
  ampSensorValue += config.AmpDigitalOffset1;

  // TODO: add ability to specify which offset to use

  if (ampSensorValue > 1024)
    ampSensorValue = 1024;

  if (ampSensorValue < 0)
    ampSensorValue = 0;

  float tmpVoltage = (ampSensorValue / 1024.0) * 5000; // Gets you mV
  return ((tmpVoltage - ACSoffset) / mVperAmp);
}

void readDht(int pin, float temp, float hum)
{
  dht.read2(&temp, &hum, NULL);
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

void updateDtm()
{
  delete(currentDtm);
  DateTime* tmpDtm = new DateTime(2021, 11, 19, 13, 27, 0);
  currentDtm = new DateTime(tmpDtm->unixtime() + uptimeSeconds);
  delete(tmpDtm);
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int getFreeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}