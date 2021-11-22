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
// <ISO_DTM>|STAT|<UptimeSeconds>|<BytesFreeMem>|<DehumEnabled>|<TelescopeOutState>|<DehumOutState>|<Aux1OutState>|<AcInState>|<BatteryCurrentStateSeconds>|<DehumCurrentStateSeconds>
// <ISO_DTM>|PWR|<Voltage>|<Battery>|<Load>|<Solar>|<AC>
// <ISO_DTM>|ENV|<CurrentTemperatureC>|<CurrentHumidityPercent>
// VERSION|<FirmwareVersion>
// CONFIG|AverageReadingCount|<value>
// CONFIG|UpdateFrequency|<value>
// CONFIG|WriteInterval|<value>
// CONFIG|VoltageCalibration|<value>
// CONFIG|R1Actual|<value>
// CONFIG|R2Actual|<value>
// CONFIG|AmpDigitalOffset1|<value>
// CONFIG|AmpDigitalOffset2|<value>
// CONFIG|AmpDigitalOffset3|<value>
// CONFIG|TemperatureCalibration|<value>
// CONFIG|HumidityCalibration|<value>
// CONFIG|TargetHumidity|<value>
// CONFIG|HumidityHysterisis|<value>
// CONFIG|AcBackupPoint|<value>
// OK
// FAIL

// Input commands:
// VERSION - Get the current firmware version
// CONFIG - Get the current controller configuration
// SET AverageReadingCount <val> - AvgRdCt -- 
// SET UpdateFrequency <val> - UdFreq -- 
// SET WriteInterval <val> - WrtInt -- 
// SET VoltageCalibration <val> - VCal -- 
// SET R1Actual <val> - R1Val -- 
// SET R2Actual <val> - R2Val -- 
// SET AmpDigitalOffset1 <val> - ADO1 -- Digital offset for the battery amperage
// SET AmpDigitalOffset2 <val> - ADO2 -- Digital offset for the load amperage
// SET AmpDigitalOffset3 <val> - ADO3 -- Digital offset for the solar amperage
// SET TemperatureCalibration <val> - TCal -- 
// SET HumidityCalibration <val> - HCal -- 
// SET TargetHumidity <val> - TgHum -- 
// SET HumidityHysterisis <val> - HumHys -- 
// SET AcBackupPoint <val> - ACBV --
// SET TIME <val in format yyyy-MM-ddTHH:mm:ss>
// ENABLE DEHUM
// DISABLE DEHUM
// PWR ON TELESCOPE
// PWR ON AUX1
// PWR ON DEHUM
// PWR ON AC
// PWR OFF TELESCOPE
// PWR OFF AUX1
// PWR OFF DEHUM
// PWR OFF AC
// SAVE - Save the current config to the EEPROM
// CLEAR - Reset the EEPROM to the default values 
// PAUSE - Pause sensor readings
// RESUME - Resume sensor readings

// Config Option conversion:
//   AverageReadingCount[19]      => AvgRdCt[7] .. 24 bytes saved
//   UpdateFrequency[15]          => UdFreq[6]  .. 18 bytes saved
//   WriteInterval[13]            => WrtInt[6]  .. 14 bytes saved
//   VoltageCalibration[18]       => VCal[4]    .. 28 bytes saved
//   R1Actual[8]                  => R1Val[5]   ..  6 bytes saved
//   R2Actual[8]                  => R2Val[5]   ..  6 bytes saved
//   AmpDigitalOffset1[17]        => ADO1[4]    .. 26 bytes saved
//   AmpDigitalOffset2[17]        => ADO2[4]    .. 26 bytes saved
//   AmpDigitalOffset3[17]        => ADO3[4]    .. 26 bytes saved
//   TemperatureCalibration[22]   => TCal[4]    .. 36 bytes saved
//   HumidityCalibration[19]      => HCal[4]    .. 30 bytes saved
//   TargetHumidity[14]           => TgHum[5]   .. 18 bytes saved
//   HumidityHysterisis[18]       => HumHys[6]  .. 24 bytes saved
//   AcBackupPoint[13]            => ACBV[4]    .. 18 bytes saved

// TOTAL: 
//        300 bytes progmem saved if implemented
//         30 bytes heap saved if implemented and serial reservation is decreased bv 15

// Serial commmand conversion
//   Version[7] => VERSION .. 14 bytes saved
//   Config[6]  => CONFIG  .. 12 bytes saved
//
//   SUBTOTAL: 26 bytes progmem saved if implemented
//
//   VERSION[7] => VER[3]  ..  8 bytes saved
//   CONFIG[6]  => CFG[3]  ..  6 bytes saved
//
//   SAVE[4]    => SAV[3]  ..  2 bytes saved
//   CLEAR[5]   => CLR[3]  ..  4 bytes saved
//   PAUSE[5]   => PSE[3]  ..  4 bytes saved
//   RESUME[6]  => RSM[3]  ..  6 bytes saved
//
//   SUBTOTAL: 30 bytes progmem saved if implemented

#pragma region Includes
//========================================================================

#include "src/Helpers.h"
#include "src/PrintHelpers.h"
#include "src/Config.h"
#include "src/State.h"

#include "src/lib/DHT/SimpleDHT.h"
#include "src/lib/RTC/RTClib.h"

#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

//========================================================================
#pragma endregion


#pragma region Defines
//========================================================================

#define ENV_WRITE_DELAY 2500
#define HUMIDITY_CHANGE_MIN_SECOND 30
#define HUMIDITY_CHECK_DELAY 7500

#define BATTERY_CHECK_DELAY 5000
#define BATTERY_CHANGE_MIN_SECOND 30

#define OVERWRITE_EEPROM

//========================================================================
#pragma endregion


#pragma region Constants
//========================================================================

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
const uint8_t PIN_TOGGLE_DEHUMIDIFIER_OUTPUT_BUTTON = 2;
const uint8_t PIN_TOGGLE_TELESCOPE_OUTPUT_BUTTON = 3;
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

//========================================================================
#pragma endregion


#pragma region Global Variables
//========================================================================

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
 * @brief Array holding all readings of voltage to be averaged
 */
float *volts = 0;

/**
 * @brief This is to ensure that we have enough readings to start sending output
 */
uint8_t readings = 0;

/**
 * @brief Debounce tracking for humidity control button
 */
int dehumButtonLastState = LOW;

/**
 * @brief Debounce tracking for telescope output button
 */
int telescopeButtonLastState = LOW;

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
    0,              // LastWriteTime
    0,              // LastReadTime
    0,              // LastTick
    0,              // LastDhtReadTime
    0,              // LastHumidityCheckTime
    0,              // LastBatteryCheckTime
    new DateTime(), // CurrentDtm
    0,              // UptimeSeconds
    0.0,            // Temperature
    0.0,            // Humidity
    0,              // DehumCurrentStateSeconds
    0,              // BatteryCurrentStateSeconds
    true,           // DehumEnabled
    false,          // DehumOutState
    false,          // TelescopeOutState
    false,          // Aux1OutState
    false,          // AcInState
};

/**
 * @brief Instance of DHT sensor object
 */
SimpleDHT22 dht(PIN_DHT);

/**
 * @brief SD Card interface instance
 */
Sd2Card card;

/**
 * @brief SD Card volume instance
 */
SdVolume volume;

/**
 * @brief SD Card filesystem root instance
 */
SdFile root;

/**
 * @brief Realtime Clock Instance
 */
RTC_PCF8523 rtc;

//========================================================================
#pragma endregion


#pragma region Setup Routines
//========================================================================

/**
 * @brief Initial board setup
 */
void setup()
{
    setupSerial();
    initConfig();
    allocateArrays();
    setupPins();
    setupSdCard();
    setupRtc();

    updateDtm();
}

/**
 * @brief Setup the serial interface
 */
void setupSerial()
{
    inputString.reserve(30);

    Serial.begin(115200);

    inputString = "";
}

/**
 * @brief Setup the SD card interface
 */
void setupSdCard()
{
    if (!SD.begin(PIN_SD_SELECT)) 
    {
        Serial.println(F("SD failed!"));

        abort();
    }
}

/**
 * @brief Abort firmware execution and flash the status light
 */
void abort()
{
    while (1)
    {
        digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, !digitalRead(PIN_DEHUMIDIFIER_ENABLED_LED));
        delay(200);
    }
}

/**
 * @brief Initialize the RTC
 */
void setupRtc()
{
    if (!rtc.begin())
    {
        Serial.println(F("RTC FAIL"));
        abort();
    }
    
    // this is only called if the time has never been set or after the battery has died
    if (!rtc.initialized() || rtc.lostPower()) 
    {
        Serial.println(F("RTC Init"));

        rtc.adjust(DateTime(2000, 1, 1, 0, 0, 0));
    }

    rtc.start();
}

/**
 * @brief Configure pins and set their initial state
 */
void setupPins()
{
    pinMode(PIN_TOGGLE_TELESCOPE_OUTPUT_BUTTON, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_DEHUMIDIFIER_OUTPUT_BUTTON, INPUT_PULLUP);

    pinMode(PIN_TELESCOPE_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_DEHUMIDIFIER_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_AUX_OUTPUT_RELAY, OUTPUT);
    pinMode(PIN_AC_INPUT_RELAY, OUTPUT);
    pinMode(PIN_DEHUMIDIFIER_ENABLED_LED, OUTPUT);

    // Synchronize the relay state with the default state of each output
    setRelayState(&state.TelescopeOutState, PIN_TELESCOPE_OUTPUT_RELAY, state.TelescopeOutState);
    setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, state.DehumOutState);
    setRelayState(&state.Aux1OutState, PIN_AUX_OUTPUT_RELAY, state.Aux1OutState);
    setRelayState(&state.AcInState, PIN_AC_INPUT_RELAY, state.AcInState);

    setHumidityControl(state.DehumEnabled);
}

/**
 * @brief Initialize the controller configuration
 */
void initConfig()
{
#ifdef OVERWRITE_EEPROM
    writeDefaultConfig();
#endif // OVERWRITE_EEPROM

    readConfig();

    // check for magic number
    if (config.Defined != -1337)
    {
        writeDefaultConfig();

        readConfig();
    }
}

//========================================================================
#pragma endregion


#pragma region Main loop
//========================================================================

/**
 * @brief Main loop
 */
void loop()
{
    processSerialInput();

    updateClock();

    if (state.EnableReadings)
    {
        readPowerSensors();
        readEnvSensors();
    }

    checkHumidity();
    checkBattery();
    readDehumButton();
    readTelescopeButton();
}

/**
 * @brief occurs whenever a new data comes in the hardware serial RX.  
 * 
 * This routine is run between each time loop() runs, so using delay 
 * inside loop can delay response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
    while (Serial.available())
    {
        // get the new byte:
        char inChar = (char)Serial.read();

        // add it to the inputString:
        if (inChar != '\n' && inChar != '\r')
            inputString += inChar;

        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n')
        {
            stringComplete = true;
            inputString += '\0';
        }
    }
}

/**
 * @brief Handle incoming serial command, if it is ready
 */
void processSerialInput()
{
    // print the string when a newline arrives:
    if (stringComplete)
    {
        handleSerialCommand(&inputString);

        // clear the string:
        inputString = "";
        stringComplete = false;
    }
}

/**
 * @brief Update all clock related variables
 */
void updateClock()
{
    // update the clock
    if (state.LastTick == 0 || (millis() - state.LastTick) >= 1000)
    {
        updateDtm();
        printSystemStatus(state, config);

        state.LastTick = millis();
    }
}

/**
 * @brief Read voltage and amperage sensors
 */
void readPowerSensors()
{
    if (state.LastReadTime == 0 || (millis() - state.LastReadTime) >= (1000 / config.UpdateFrequency))
    {
        pushReading(volts, getVoltage(PIN_VOLTAGE));
        pushReading(amps_battery, getAmperage(PIN_BATT_AMPERAGE, config.AmpDigitalOffset1));
        pushReading(amps_load, (getAmperage(PIN_LOAD_AMPERAGE, config.AmpDigitalOffset2)*-1));
        pushReading(amps_solar, (getAmperage(PIN_SOLAR_AMPERAGE, config.AmpDigitalOffset3)*-1));

        state.Volt = getAvgReading(volts, config.AverageReadingCount);
        state.BatteryAmp = getAvgReading(amps_battery, config.AverageReadingCount);
        state.LoadAmp = getAvgReading(amps_load, config.AverageReadingCount);
        state.SolarAmp = getAvgReading(amps_solar, config.AverageReadingCount);
        state.AcAmp = state.BatteryAmp + state.LoadAmp - state.SolarAmp;

        // increment the reading counter if it is less than the number we need to average
        if (readings < config.AverageReadingCount)
            readings++;

        state.LastReadTime = millis();

        // we only want to send the current values periodically, even though we refresh internally multiple times per second
        if (state.LastWriteTime == 0 || ((millis() - state.LastWriteTime) >= (config.WriteInterval * 1000)))
        {
            // only write if we have enough readings to average
            if (readings == config.AverageReadingCount)
            {
                printPowerStatus(state, config);

                state.LastWriteTime = millis();
            }
        }
    }
}

/**
 * @brief Read temperature/humidity sensors
 */
void readEnvSensors()
{
    if (state.LastDhtReadTime == 0 || (millis() - state.LastDhtReadTime) >= ENV_WRITE_DELAY)
    {
        readDht(PIN_DHT, &state.Temperature, &state.Humidity);
        printEnvironmentStatus(state, config);

        state.LastDhtReadTime = millis();
    }
}

/**
 * @brief Routine that is run to check if humidity control needs to be turned on or off
 */
void checkHumidity()
{
    // if humidity control is disabled, but the output is currently on
    if (state.DehumEnabled == false && state.DehumOutState == true)
    {
        setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, false);
        state.DehumCurrentStateSeconds = 0;
    }

    // we don't check the humidity level if automatic humidity control has been disabled
    if (state.DehumEnabled == false)
        return;

    // we don't check the humidity level if the state has changed since the minimum threshold
    if (state.DehumCurrentStateSeconds < HUMIDITY_CHANGE_MIN_SECOND)
        return;

    if (state.LastHumidityCheckTime == 0 || (millis() - state.LastHumidityCheckTime) > HUMIDITY_CHECK_DELAY)
    {
        uint8_t onThreshold = config.TargetHumidity + (config.HumidityHysterisis / 2);
        uint8_t offThreshold = config.TargetHumidity - (config.HumidityHysterisis / 2);

        if (state.Humidity >= onThreshold)
        {
            setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, true);
            state.DehumCurrentStateSeconds = 0;
        }
        else if (state.Humidity <= offThreshold)
        {
            setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, false);
            state.DehumCurrentStateSeconds = 0;
        }

        state.LastHumidityCheckTime = millis();
    }
}

/**
 * @brief Routine that is run to check if the battery charge level has dropped low enough to 
 * require the AC backup to be engaged
 */
void checkBattery()
{
    // nothing to do right now, this is a placeholder for after the 
    // capacity tracking is enabled
}

/**
 * @brief Check for button press on the humidity control button
 */ 
void readDehumButton()
{
    int currentState = digitalRead(PIN_TOGGLE_DEHUMIDIFIER_OUTPUT_BUTTON);

    // state has changed
    if (currentState != dehumButtonLastState)
    {
        // button is pressed
        if (currentState == LOW)
            setHumidityControl(!state.DehumEnabled);

        delay(20);
        dehumButtonLastState = currentState;
    }
}

/**
 * @brief Check for button press on the telescope output button
 */
void readTelescopeButton()
{
    int currentState = digitalRead(PIN_TOGGLE_TELESCOPE_OUTPUT_BUTTON);

    // state has changed
    if (currentState != telescopeButtonLastState)
    {
        // button is pressed
        if (currentState == LOW)
            setRelayState(&state.TelescopeOutState, PIN_TELESCOPE_OUTPUT_RELAY, !state.TelescopeOutState);

        delay(20);
        telescopeButtonLastState = currentState;
    }
}

/**
 * @brief Helper used for setting the humidity control state and updating the LED to match
 */
void setHumidityControl(bool newState)
{
    state.DehumEnabled = newState;
    digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, state.DehumEnabled);
}

//========================================================================
#pragma endregion


#pragma region Serial command handling
//========================================================================

/**
 * @brief Handle commands being received via serial connection
 */
void handleSerialCommand(String *commandLine)
{
    boolean fail = false;

    if (commandLine->startsWith(F("PWR")))
    {
        *commandLine = commandLine->substring(4);

        byte cmd = 250;

        if (commandLine->startsWith(F("ON")))
        {
            *commandLine = commandLine->substring(3);
            cmd = 1;
        }

        if (commandLine->startsWith(F("OFF")))
        {
            *commandLine = commandLine->substring(4);
            cmd = 0;
        }

        if (cmd <= 1)
        {
            if (commandLine->startsWith(F("TELESCOPE")))
                setRelayState(&state.TelescopeOutState, PIN_TELESCOPE_OUTPUT_RELAY, (cmd == 1));
            else if (commandLine->startsWith(F("AUX1")))
                setRelayState(&state.Aux1OutState, PIN_AUX_OUTPUT_RELAY, (cmd == 1));
            else if (commandLine->startsWith(F("DEHUM")))
                setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, (cmd == 1));
            else if (commandLine->startsWith(F("AC")))
                setRelayState(&state.AcInState, PIN_AC_INPUT_RELAY, (cmd == 1));
            else
                fail = true;
        }
        else
            fail = true;
    }
    else if (commandLine->startsWith(F("SET")))
    {
        *commandLine = commandLine->substring(4);

        if (commandLine->startsWith(F("TIME")))
        {
            *commandLine = commandLine->substring(5);

            // 2020-06-25T15:29:37
            char dtmStr[20];
            memset(dtmStr, 0, 20);
            commandLine->toCharArray(dtmStr, 20, 0);         

            rtc.adjust(DateTime(dtmStr));
        }

        // 2021-11-21T17:41:33|PWR|12.92|-0.05|0.04|0.03|-0.04

        else if (parseConfigValShort(commandLine, &config.AverageReadingCount, F("AverageReadingCount"), 20)) 
            reallocateArrays();
        else if (parseConfigValShort(commandLine, &config.UpdateFrequency, F("UpdateFrequency"), 16)) { }
        else if (parseConfigValShort(commandLine, &config.WriteInterval, F("WriteInterval"), 14)) { }
        else if (parseConfigValFloat(commandLine, &config.VoltageCalibration, F("VoltageCalibration"), 19)) { }
        else if (parseConfigValInt(commandLine, &config.R1Actual, F("R1Actual"), 9)) { }
        else if (parseConfigValInt(commandLine, &config.R2Actual, F("R2Actual"), 9)) { }
        else if (parseConfigValShort(commandLine, &config.AmpDigitalOffset1, F("AmpDigitalOffset1"), 17)) { }
        else if (parseConfigValShort(commandLine, &config.AmpDigitalOffset2, F("AmpDigitalOffset2"), 17)) { }
        else if (parseConfigValShort(commandLine, &config.AmpDigitalOffset3, F("AmpDigitalOffset3"), 17)) { }
        else if (parseConfigValFloat(commandLine, &config.TemperatureCalibration, F("TemperatureCalibration"), 23)) { }
        else if (parseConfigValFloat(commandLine, &config.HumidityCalibration, F("HumidityCalibration"), 20)) { }
        else if (parseConfigValShort(commandLine, &config.TargetHumidity, F("TargetHumidity"), 15)) { }
        else if (parseConfigValShort(commandLine, &config.HumidityHysterisis, F("HumidityHysterisis"), 19)) { }
        else if (parseConfigValFloat(commandLine, &config.AcBackupPoint, F("AcBackupPoint"), 14)) { }
    }

    else if (commandLine->startsWith(F("CONFIG")))
        printConfig(config);

    else if (commandLine->startsWith(F("VERSION")))
        printPipePair(F("VERSION"), VERSION, true);

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
        fail = true;

    if (fail)
        Serial.println(F("FAIL"));
}

/**
 * @brief Parse an incoming serial config floating point value
 */
boolean parseConfigValFloat(String *commandLine, float *destination, const __FlashStringHelper *itemName, uint8_t length)
{
    if (!commandLine->startsWith(itemName))
        return false;

    *commandLine = commandLine->substring(length);
    *destination = commandLine->toFloat();

    printConfigEntry(itemName, *destination);

    return true;
}

/**
 * @brief Parse an incoming serial config floating point value
 */
boolean parseConfigValShort(String *commandLine, uint8_t *destination, const __FlashStringHelper *itemName, uint8_t length)
{
    if (!commandLine->startsWith(itemName))
        return false;

    *commandLine = commandLine->substring(length);
    *destination = commandLine->toInt();

    printConfigEntry(itemName, *destination);

    return true;
}

/**
 * @brief Parse an incoming serial config floating point value
 */
boolean parseConfigValInt(String *commandLine, uint16_t *destination, const __FlashStringHelper *itemName, uint8_t length)
{
    if (!commandLine->startsWith(itemName))
        return false;

    *commandLine = commandLine->substring(length);
    *destination = commandLine->toInt();

    printConfigEntry(itemName, (uint32_t)*destination);

    return true;
}

//========================================================================
#pragma endregion


#pragma region Config
//========================================================================

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
    Serial.println(F("WRITE default config"));
    EEPROM.put(0, getDefaultConfig());
}

//========================================================================
#pragma endregion


#pragma region Utiltiies
//========================================================================

/**
 * @brief Called once during board startup to allocate space for reading arrays
 */
void allocateArrays()
{
    readings = 0;

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

    allocateArrays();
}

/**
 * @brief Push a new reading to the array of readings for averaging purposes
 */
void pushReading(float *readingArray, float newValue)
{
    // work backwards through the array, moving the contents back by one element
    for (int i = config.AverageReadingCount - 2; i >= 0; i--)
        readingArray[i + 1] = readingArray[i];

    // set the first value to the new provided value
    readingArray[0] = newValue;
}

/**
 * @brief Update the time from the RTC
 */
void updateDtm()
{
    *state.CurrentDtm = rtc.now();

    // TODO: is there a way to set an alarm on the RTC and then read it in software to see if it has triggered.. to ensure 1 second has actually passed vs using millis()
    state.UptimeSeconds = millis() / 1000;
    state.DehumCurrentStateSeconds += 1;
    state.BatteryCurrentStateSeconds += 1;
}

//========================================================================
#pragma endregion


#pragma region Sensor and Relay Support
//========================================================================

/**
 * @brief Set the state of the named relay and store it's new state
 */
void setRelayState(boolean *target, uint8_t pin, boolean newState)
{
    *target = newState;

    digitalWrite(pin, !newState);
}

/**
 * @brief Read the voltage on the specified pin
 */
float getVoltage(int pin)
{
    int voltSensorValue = analogRead(pin);

    float pinVoltage = ((float)voltSensorValue / 1024.0) * 5.0;

    float denominator = (float)config.R1Actual / (config.R1Actual + config.R2Actual);

    return ((float)pinVoltage / denominator) + config.VoltageCalibration;
}

/**
 * @brief Read the amperage on the specified pin
 */
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

/**
 * @brief Read the Temperature and Humdiity from the DHT22 sensor
 */
void readDht(int pin, float *temp, float *hum)
{
    dht.read2(temp, hum, NULL);
}

//========================================================================
#pragma endregion