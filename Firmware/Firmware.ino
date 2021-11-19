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

#include "src/Helpers.h"
#include "src/PrintHelpers.h"
#include "src/Config.h"
#include "src/State.h"

#include "src/lib/DHT/SimpleDHT.h"
#include "src/lib/RTC/RTClib.h"

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
 * @brief Array holding all readings of battery amperage to be averaged
 */
float *amps_battery = 0;

/**
 * @brief Array holding all readings of load amperage to be averaged
 */
float *amps_load = 0;

/**
 * @brief Array holding all readings of solar amperage to be averaged
 */
float *amps_solar = 0;

/**
 * @brief Array holding all readings of AC amperage to be averaged
 */
float *amps_ac = 0;

/**
 * @brief Array holding all readings of voltage to be averaged
 */
float *volts = 0;

/**
 * @brief String object used to hold input read from serial port
 */
String inputString = "";

/**
 * @brief If true, the string has finished being read from the serial port
 */
bool stringComplete = false;

/**
 * @brief Object used to hold current configuration
 */
ConfigObject config;

/**
 * @brief Object containing current state of controller
 */
State state = {
    true,           // EnableReadings
    false,          // TelescopeOutputEnabled
    0,              // LastWriteTime
    0,              // LastReadTime
    0,              // LastTick
    0,              // LastDhtReadTime
    new DateTime(), // CurrentDtm
    0,              // UptimeSeconds
    0.0,            // Temperature
    0.0,            // Humidity
};

/**
 * @brief Instance of DHT sensor object
 */
SimpleDHT22 dht(PIN_DHT);

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

    initConfig();
    allocateArrays();
    setupPins();

    // TODO: Remove from final code
    state.CurrentDtm = new DateTime(2021, 11, 19, 13, 27, 0);
}

void setupPins()
{
    pinMode(PIN_TELESCOPE_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_DEHUMIDIFIER_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_AUX_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_AC_INPUT_RELAY, OUTPUT);
    pinMode(PIN_DEHUMIDIFIER_ENABLED_LED, OUTPUT);

    digitalWrite(PIN_TELESCOPE_OUTPUT_RELAY, LOW);
    digitalWrite(PIN_DEHUMIDIFIER_OUTPUT_RELAY, HIGH);
    digitalWrite(PIN_AUX_OUTPUT_RELAY, HIGH);
    digitalWrite(PIN_AC_INPUT_RELAY, HIGH);
}

void initConfig()
{
    readConfig();

    // check for magic number
    if (config.Defined != -1337)
    {
        writeDefaultConfig();

        readConfig();
    }
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

    // update the clock
    if (state.LastTick == 0 || (millis() - state.LastTick) >= 1000)
    {
        state.UptimeSeconds = millis() / 1000;
        updateDtm();

        // TODO: For testing only.. remove for final code
        digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, !digitalRead(PIN_DEHUMIDIFIER_ENABLED_LED));
        // digitalWrite(PIN_TELESCOPE_OUTPUT_RELAY, !digitalRead(PIN_TELESCOPE_OUTPUT_RELAY));

        printSystemStatus(state, config);

        state.LastTick = millis();
    }

    if (state.EnableReadings)
    {
        if (state.LastReadTime == 0 || (millis() - state.LastReadTime) >= config.UpdateFrequency)
        {
            pushReading(volts, getVoltage(PIN_VOLTAGE));
            pushReading(amps_battery, getAmperage(PIN_BATT_AMPERAGE, config.AmpDigitalOffset1));
            pushReading(amps_load, getAmperage(PIN_LOAD_AMPERAGE, config.AmpDigitalOffset2));
            pushReading(amps_solar, getAmperage(PIN_SOLAR_AMPERAGE, config.AmpDigitalOffset3));

            // TODO: calculate AC amperage
            pushReading(amps_ac, 0.0);

            state.LastReadTime = millis();
        }

        if (state.LastDhtReadTime == 0 || (millis() - state.LastDhtReadTime) >= ENV_WRITE_DELAY)
        {
            readDht(PIN_DHT, state.Temperature, state.Humidity);
            printEnvironmentStatus(state, config);

            state.LastDhtReadTime = millis();
        }

        // we only want to send the current values periodically, even though we refresh internally multiple times per second
        if (state.LastWriteTime == 0 || ((millis() - state.LastWriteTime) >= (config.WriteInterval * 1000)))
        {
            printVoltStatus(state, config, volts);
            printAmpStatus(state, config, amps_battery, F("BATT"));
            printAmpStatus(state, config, amps_load, F("LOAD"));
            printAmpStatus(state, config, amps_solar, F("SOLAR"));
            printAmpStatus(state, config, amps_ac, F("AC"));

            state.LastWriteTime = millis();
        }
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
    amps_ac = new float[config.AverageReadingCount];

    initializeArray(volts);
    initializeArray(amps_battery);
    initializeArray(amps_load);
    initializeArray(amps_solar);
    initializeArray(amps_ac);
}

/**
 * @brief Initializse the specified array with all empty values
 */
void initializeArray(float *readingArray)
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
        delete[] amps_battery;

    if (volts != 0)
        delete[] volts;

    if (amps_load != 0)
        delete[] amps_load;

    if (amps_solar != 0)
        delete[] amps_solar;

    if (amps_ac != 0)
        delete[] amps_ac;

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
    EEPROM.put(0, getDefaultConfig());
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

void handleSerialCommand(String *commandLine)
{
    if (commandLine->startsWith(F("GET")))
    {
        // strip the GET from the beginning
        *commandLine = commandLine->substring(4);

        if (commandLine->startsWith(F("Version")))
            printPipePair(F("VERSION"), VERSION, true);

        else if (commandLine->startsWith(F("Config")))
            printConfig(config);
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
        Serial.println(F("OK"));
    }

    else if (commandLine->startsWith(F("CLEAR")))
    {
        writeDefaultConfig();
        readConfig();
        Serial.println(F("OK"));
    }

    else if (commandLine->startsWith(F("PAUSE")))
    {
        state.EnableReadings = false;
        Serial.println(F("OK"));
    }

    else if (commandLine->startsWith(F("RESUME")))
    {
        state.EnableReadings = true;
        Serial.println(F("OK"));
    }

    else
    {
        // TODO: do something with invalid commands?
        Serial.println(F("FAIL"));
    }
}

float getVoltage(int pin)
{
    int voltSensorValue = analogRead(pin);

    float pinVoltage = ((float)voltSensorValue / 1024.0) * 5.0;

    float denominator = (float)config.R1Actual / (config.R1Actual + config.R2Actual);

    return ((float)pinVoltage / denominator) + config.VoltageCalibration;
}

float getAmperage(int pin, int offset)
{
    int ampSensorValue = analogRead(pin);

    // calibration logic.. rather rudimentary
    ampSensorValue += offset;

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

void pushReading(float *readingArray, float newValue)
{
    // work backwards through the array, moving the contents back by one element
    for (int i = config.AverageReadingCount - 2; i >= 0; i--)
        readingArray[i + 1] = readingArray[i];

    // set the first value to the new provided value
    readingArray[0] = newValue;
}

void updateDtm()
{
    delete (state.CurrentDtm);
    DateTime *tmpDtm = new DateTime(2021, 11, 19, 13, 27, 0);
    state.CurrentDtm = new DateTime(tmpDtm->unixtime() + state.UptimeSeconds);
    delete (tmpDtm);
}
