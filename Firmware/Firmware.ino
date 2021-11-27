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
// <ISO_DTM>|<UptimeSeconds>|STAT|<BytesFreeMem>|<DehumEnabled>|<TelescopeOutState>|<DehumOutState>|<Aux1OutState>|<AcInState>|<BatteryCurrentStateSeconds>|<DehumCurrentStateSeconds>|<PcConnected>|<LastPingSeconds>
// <ISO_DTM>|<UptimeSeconds>|PWR|<Voltage>|<BatteryAmps>|<LoadAmps>|<SolarAmps>|<AcAmps>|<BatterySoc>|<BatteryCapacityAh>
// <ISO_DTM>|<UptimeSeconds>|ENV|<CurrentTemperatureC>|<CurrentHumidityPercent>
// <ISO_DTM>|<UptimeSeconds>|VERSION|<FirmwareVersion>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AverageReadingCount|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|UpdateFrequency|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|WriteInterval|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|VoltageCalibration|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|R1Actual|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|R2Actual|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AmpDigitalOffset1|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AmpDigitalOffset2|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AmpDigitalOffset3|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|TemperatureCalibration|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|HumidityCalibration|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|TargetHumidity|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|HumidityHysterisis|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AcBackupPoint|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|AcRecoveredPoint|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|BatteryCapacityAh|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|BatteryEndingAmps|<value>
// <ISO_DTM>|<UptimeSeconds>|CONFIG|BatteryAbsorbVoltage|<value>
// <ISO_DTM>|<UptimeSeconds>|<cmd>|OK
// <ISO_DTM>|<UptimeSeconds>|<cmd>|FAIL
// <ISO_DTM>|<UptimeSeconds>|OPEN
// <ISO_DTM>|<UptimeSeconds>|CLOSE
// HIST:<LogLine>

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
// SET AcBackupPoint <val> - ACBP --
// SET AcRecoveredPoint <val> - ACBP --
// SET BatteryCapacityAh <val> -  -- 
// SET BatteryEndingAmps <val> -  -- 
// SET BatteryAbsorbVoltage <val> -  -- 
// SET TIME <val in format yyyy-MM-ddTHH:mm:ss>
// SAVE - Save the current config to the EEPROM
// RESET CONFIG -- Re-read the config from the EEPROM
// RESET EEPROM -- Reset the EEPROM to default values
// RESET LOG -- Clear the log file on the SD card
// RESET SOC -- Reset the SOC to 100%
// DEHUM ON
// DEHUM OFF
// PWR ON TELESCOPE
// PWR ON AUX1
// PWR ON DEHUM
// PWR ON AC
// PWR OFF TELESCOPE
// PWR OFF AUX1
// PWR OFF DEHUM
// PWR OFF AC
// PAUSE - Pause sensor readings
// RESUME - Resume sensor readings
// PING - Notify the controller that the PC is still alive
// PC OFF - Notify the controller that the PC is no longer connected
// PC ON - Notify the controller that the PC is now connected
// READ - Dump any historical log lines that are stored on the controller, then clear the log history

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
//   AcBackupPoint[13]            => ACBP[4]    .. 18 bytes saved
//   AcRecoveredPoint[16]         => ACRP[4]    .. 28 bytes saved
//   BatteryCapacityAh[17]        => BCA[3]     .. 28 bytes saved
//   BatteryEndingAmps[17]        => BEA[3]     .. 28 bytes saved
//   BatteryAbsorbVoltage[20]     => BAV[3]     .. 34 bytes saved

// TOTAL: 
//        390 bytes progmem saved if implemented
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

// Possible to-do items:
//  config validation?
//  store AC backup state to SD?

// To strip if necessary to free more code space:
//  pause/resume functionality

#pragma region Includes
//========================================================================

#include "src/Helpers.h"
#include "src/PrintHelpers.h"
#include "src/Config.h"
#include "src/State.h"
#include "src/Strings.h"

#include "src/lib/DHT/SimpleDHT.h"
#include "src/lib/RTC/RTClib.h"

#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

#include <avr/pgmspace.h>

//========================================================================
#pragma endregion


#pragma region Defines
//========================================================================

#define ENV_WRITE_DELAY 2500
#define HUMIDITY_CHANGE_MIN_SECOND 30
#define HUMIDITY_CHECK_DELAY 7500

#define BATTERY_CHECK_DELAY 5000
#define BATTERY_CHANGE_MIN_SECOND 30

#define MAX_PING_TIME 15
#define MAX_DUMP_MS_PER_CYCLE 300

#define SOC_STORE_FREQUENCY_SECONDS 30

// #define OVERWRITE_EEPROM

//========================================================================
#pragma endregion


#pragma region Constants
//========================================================================

const char VERSION[] = "2.0";

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
byte dehumButtonLastState = LOW;

/**
 * @brief Debounce tracking for telescope output button
 */
byte telescopeButtonLastState = LOW;

/**
 * @brief Array to handle input commands from serial port
 */
char serialInput[35];

/**
 * @brief If true, the string has finished being read from the serial port
 */
bool stringComplete = false;

// /**
//  * @brief Number of minutes that have elapsed since midnight
//  */
// uint16_t minutesSinceMidnight = 1400;

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
    0,              // LastBatterySnapshotTime
    new DateTime(), // CurrentDtm
    0,              // UptimeSeconds
    0.0,            // Temperature
    0.0,            // Humidity
    0,              // DehumCurrentStateSeconds
    0,              // BatteryCurrentStateSeconds
    0,              // LastPingSeconds
    true,           // DehumEnabled
    false,          // DehumOutState
    false,          // TelescopeOutState
    false,          // Aux1OutState
    false,          // AcInState
    false,          // PcConnected
    100,            // BatterySoc
    0.0,            // BatteryCapacityAh
    0.0             // BatteryCapacityMicroAH
};

/**
 * @brief Instance of DHT sensor object
 */
SimpleDHT22 dht(PIN_DHT);

/**
 * @brief Handle for the log file on the SD card
 */
File logFile;

/**
 * @brief  Name of the log file on the SD card
 */
char logFileName[8];

/**
 * @brief Realtime Clock Instance
 */
RTC_PCF8523 rtc;

/**
 * @brief Current index of the serial input
 */
uint8_t currentSerialPos = 0;

/**
 * @brief If true, we are currently being instructed to dump the historical log file
 */
bool doReadLog = false;

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
    setupRtc();
    setupSdCard();
    readLastSoc();

    setPcDisconnected();

    setPulse(6, 75);
}

void readLastSoc()
{
    readSocFromSd();

    if (state.BatteryCapacityMicroAH == 0)
        resetSoc();
}

/**
 * @brief Setup the serial interface
 */
void setupSerial()
{
    Serial.begin(115200);

    memset(serialInput, 0, sizeof(serialInput));
}

/**
 * @brief Setup the SD card interface
 */
void setupSdCard()
{
    if (!SD.begin(PIN_SD_SELECT)) 
    {
        writeLine_p(Serial, STR_SD_FAILED);
        panic();
    }

    setLogFileName(false);
}

/**
 * @brief Panic program execution and flash the status light
 */
void panic()
{
    while (1)
    {
        digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, !digitalRead(PIN_DEHUMIDIFIER_ENABLED_LED));
        delay(100);
    }
}

/**
 * @brief Initialize the RTC
 */
void setupRtc()
{
    if (!rtc.begin())
    {
        writeLine_p(Serial, STR_RTC_FAIL);
        panic();
    }
    
    // this is only called if the time has never been set or after the battery has died
    if (!rtc.initialized() || rtc.lostPower()) 
    {
        writeLine_p(getPrintTarget(), STR_RTC_INIT);

        rtc.adjust(DateTime(2000, 1, 1, 0, 0, 0));
    }

    rtc.start();
    updateDtm();
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
        writeDefaultConfig();
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
    checkPing();
    pulseLed();
    readLog();
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

        // for some reason, on first boot after an upload.. the first character that
        // comes in is invalid and if you attempt to print it back to serial, it jacks
        // up the serial connection somehow.. easy enough to filter out since it seems 
        // to send a negative char value
        if ((int)inChar < 0)
            continue;

        // if the input string is longer than the buffer, we truncate it and process 
        // the command we do have
        if (currentSerialPos >= (sizeof(serialInput)-1))
        {
            stringComplete = true;
            break;
        }

        // add it to the inputString:
        if (inChar != '\n' && inChar != '\r')
            serialInput[currentSerialPos++] = inChar;

        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n')
        {
            stringComplete = true;
            serialInput[currentSerialPos++] = '\0';
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
        handleSerialCommand(serialInput);

        // clear the string:
        currentSerialPos = 0;
        memset(serialInput, 0, sizeof(serialInput));
        stringComplete = false;
    }
}

/**
 * @brief Update all clock related variables
 */
void updateClock()
{
    // update the clock and print the system status
    if (state.LastTick == 0 || (uint32_t)(millis() - state.LastTick) >= 1000)
    {
        updateDtm();
        printSystemStatus(getPrintTarget(), state, config);
        flushLog();

        state.LastTick = millis();
    }
}

/**
 * @brief Read voltage and amperage sensors
 */
void readPowerSensors()
{
    if (state.LastReadTime == 0 || (uint32_t)(millis() - state.LastReadTime) >= (1000 / config.UpdateFrequency))
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
        if (state.LastWriteTime == 0 || ((uint32_t)(millis() - state.LastWriteTime) >= (config.WriteInterval * 1000)))
        {
            // only write if we have enough readings to average
            if (readings == config.AverageReadingCount)
            {
                printPowerStatus(getPrintTarget(), state, config);
                flushLog();

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
    if (state.LastDhtReadTime == 0 || (uint32_t)(millis() - state.LastDhtReadTime) >= ENV_WRITE_DELAY)
    {
        readDht(PIN_DHT, &state.Temperature, &state.Humidity);
        printEnvironmentStatus(getPrintTarget(), state, config);
        flushLog();

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

    if (state.LastHumidityCheckTime == 0 || (uint32_t)(millis() - state.LastHumidityCheckTime) > HUMIDITY_CHECK_DELAY)
    {
        uint8_t onThreshold = config.TargetHumidity + (config.HumidityHysterisis / 2);
        uint8_t offThreshold = config.TargetHumidity - (config.HumidityHysterisis / 2);

        if (state.Humidity >= onThreshold && !state.DehumOutState)
        {
            setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, true);
            state.DehumCurrentStateSeconds = 0;
        }
        else if (state.Humidity <= offThreshold && state.DehumOutState)
        {
            setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, false);
            state.DehumCurrentStateSeconds = 0;
        }

        state.LastHumidityCheckTime = millis();
    }
}

/**
 * @brief Routine that monitors the state of the battery
 */
void checkBattery()
{
    // lets not do anything until we have sufficient readings
    if (readings < config.AverageReadingCount)
        return;

    // we want to synchronize the battery capacity tracking with the 
    // reading averaging period.. this can be found by dividing 
    // config.AverageReadingCount by config.UpdateFrequency
    //
    // for example:
    // if AverageReadingCount is 10 and UpdateFrequency is 2
    // 10/2 = 5
    //
    // This means we should update the battery capacity every 5 seconds..
    //
    if (state.LastBatteryCheckTime == 0 || (uint32_t)(millis() - state.LastBatteryCheckTime) >= (config.AverageReadingCount/config.UpdateFrequency) * 1000)
    {
        uint32_t updateMillis = millis();
        uint32_t elapsed = updateMillis - state.LastBatteryCheckTime;
        float factor = (elapsed / (float)3600000) * 10000000.0;
        float avgReading = getAvgReading(amps_battery, config.AverageReadingCount);
        int32_t adjust = round(factor * avgReading);

        state.BatteryCapacityMicroAH += adjust;

        bool forceFull = false;

        if (state.BatteryCapacityMicroAH > config.BatteryCapacityAh * 10000000UL)
            forceFull = true;

        // if we reaching the ending amps, then we can reset SOC to 100%
        if (state.BatteryAmp > 0 && state.BatteryAmp <= config.BatteryEndingAmps && state.Volt >= config.BatteryAbsorbVoltage)
            forceFull = true;

        if (forceFull)
            state.BatteryCapacityMicroAH = config.BatteryCapacityAh * 10000000UL;

        state.BatteryCapacityAh = (float)state.BatteryCapacityMicroAH / 10000000.0;
        state.BatterySoc = ((float)state.BatteryCapacityMicroAH / ((float)config.BatteryCapacityAh * 10000000.0)) * 100.0;
        state.LastBatteryCheckTime = updateMillis;

        if (state.BatteryCurrentStateSeconds > BATTERY_CHANGE_MIN_SECOND)
        {
            if (!state.AcInState && state.BatterySoc < config.AcBackupPoint)
            {
                setRelayState(&state.AcInState, PIN_AC_INPUT_RELAY, true);
                state.BatteryCurrentStateSeconds = 0;
            }
            else if (state.AcInState && state.BatterySoc >= config.AcRecoveredPoint)
            {
                setRelayState(&state.AcInState, PIN_AC_INPUT_RELAY, false);
                state.BatteryCurrentStateSeconds = 0;
            }
        }

        writeSocToSd();
    }
}

void writeSocToSd()
{
    // our timer has elapsed.. lets store the current value
    if ((uint32_t)(millis() - state.LastBatterySnapshotTime) >= (SOC_STORE_FREQUENCY_SECONDS * 1000))
    {
        bool needToCloseLogfile = (logFile == true);

        if (needToCloseLogfile)
            closeLogFile(false);

        // switch to the soc.log file
        setLogFileName(true);

        // open the soc.log file
        openLogFile(false, true);

        // write the current SOC from state.BatteryCapacityMicroAH to the file
        logFile.println(state.BatteryCapacityMicroAH);
        logFile.flush();

        // close the soc.log file
        closeLogFile(false);

        // revert back to the tpm.log file
        setLogFileName(false);

        if (needToCloseLogfile)
            openLogFile(false, false);

        state.LastBatterySnapshotTime = millis();

        setPulse(4, 150);
    }
}

/**
 * @brief Read the SOC from the SD card
 * 
 * @warning THIS SHOULD ONLY BE CALLED FROM setup() !!!
 */
void readSocFromSd()
{
    // switch to the soc.log file
    setLogFileName(true);

    // open the soc.log file
    openLogFile(false, false);

    // we only pull the SOC if there is a file
    if (logFile)
    {
        logFile.seek(0);

        uint8_t index = 0;

        // clear the serial buffer
        memset(serialInput, '\0', sizeof(serialInput));

        while (logFile.available()) 
        {
            int character = logFile.read();

            if (character == '\n')
                break;

            serialInput[index++] = (char)character;
        }

        char *ptr;

        state.BatteryCapacityMicroAH = strtoul(serialInput, &ptr, 10);

        if (state.BatteryCapacityMicroAH > config.BatteryCapacityAh * 10000000UL)
            state.BatteryCapacityMicroAH = config.BatteryCapacityAh * 10000000UL;

        state.BatteryCapacityAh = (float)state.BatteryCapacityMicroAH / 10000000.0;
        state.BatterySoc = ((float)state.BatteryCapacityMicroAH / ((float)config.BatteryCapacityAh * 10000000.0)) * 100.0;

        memset(serialInput, '\0', sizeof(serialInput));

        // close the soc.log file
        closeLogFile(false);
    }

    // revert back to the tpm.log file
    setLogFileName(false);
    
    // We want to "start the clock" on the battery snapshot time immediately
    // as opposed to writing the SOC immediately. Writing it on first cycle 
    // makes no sense because we just read it in setup()
    if (state.LastBatterySnapshotTime == 0)
        state.LastBatterySnapshotTime = millis();
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

/**
 * @brief Check if the time since last ping has exceeded the threshold, if so
 * mark the PC as disconnected
 */
void checkPing()
{
    if (state.LastPingSeconds > MAX_PING_TIME && state.PcConnected)
    {
        setPcDisconnected();
        state.PcConnected = false;
    }
}

//========================================================================
#pragma endregion


#pragma region Serial command handling
//========================================================================

/**
 * @brief Handle commands being received via serial connection
 */
void handleSerialCommand(const char *command)
{
    setPcConnected();

    bool fail = false;

    uint8_t offset = 0;

    if (startsWith_p(command, STR_READ, offset))
        startLogDump();

    else if (startsWith_p(command, STR_PWR, offset))
    {
        offset += strlen_P(STR_PWR) + 1;

        byte cmd = 250;

        if (startsWith_p(command, STR_ON, offset))
        {
            offset += strlen_P(STR_ON) + 1;
            cmd = 1;
        }

        if (startsWith_p(command, STR_OFF, offset))
        {
            offset += strlen_P(STR_OFF) + 1;
            cmd = 0;
        }

        if (cmd <= 1)
        {
            if (startsWith_p(command, STR_TELESCOPE, offset))
                setRelayState(&state.TelescopeOutState, PIN_TELESCOPE_OUTPUT_RELAY, (cmd == 1));
            else if (startsWith_p(command, STR_AUX1, offset))
                setRelayState(&state.Aux1OutState, PIN_AUX_OUTPUT_RELAY, (cmd == 1));
            else if (startsWith_p(command, STR_DEHUM, offset))
                setRelayState(&state.DehumOutState, PIN_DEHUMIDIFIER_OUTPUT_RELAY, (cmd == 1));
            else if (startsWith_p(command, STR_AC, offset))
                setRelayState(&state.AcInState, PIN_AC_INPUT_RELAY, (cmd == 1));
            else
                fail = true;
        }
        else
            fail = true;
    }
    else if (startsWith_p(command, STR_SET, offset))
    {
        offset += strlen_P(STR_SET) + 1;

        if (startsWith_p(command, STR_TIME, offset))
        {
            offset += strlen_P(STR_TIME) + 1;

            rtc.adjust(DateTime(command + offset));
        }

        else if (parseConfigValByte_p(command, &config.AverageReadingCount, STR_AVERAGE_READING_COUNT, offset))
            reallocateArrays();
        else if (parseConfigValByte_p(command, &config.UpdateFrequency, STR_UPDATE_FREQUENCY, offset)) { }
        else if (parseConfigValByte_p(command, &config.WriteInterval, STR_WRITE_INTERVAL, offset)) { }
        else if (parseConfigValFloat_p(command, &config.VoltageCalibration, STR_VOLTAGE_CALIBRATION, offset)) { }
        else if (parseConfigValInt_p(command, &config.R1Actual, STR_R1_ACTUAL, offset)) { }
        else if (parseConfigValInt_p(command, &config.R2Actual, STR_R2_ACTUAL, offset)) { }
        else if (parseConfigValShort_p(command, &config.AmpDigitalOffset1, STR_AMP_DIGITAL_OFFSET_1, offset)) { }
        else if (parseConfigValShort_p(command, &config.AmpDigitalOffset2, STR_AMP_DIGITAL_OFFSET_2, offset)) { }
        else if (parseConfigValShort_p(command, &config.AmpDigitalOffset3, STR_AMP_DIGITAL_OFFSET_3, offset)) { }
        else if (parseConfigValFloat_p(command, &config.TemperatureCalibration, STR_TEMPERATURE_CALIBRATION, offset)) { }
        else if (parseConfigValFloat_p(command, &config.HumidityCalibration, STR_HUMIDITY_CALIBRATION, offset)) { }
        else if (parseConfigValByte_p(command, &config.TargetHumidity, STR_TARGET_HUMIDITY, offset)) { }
        else if (parseConfigValByte_p(command, &config.HumidityHysterisis, STR_HUMIDITY_HYSTERISIS, offset)) { }
        else if (parseConfigValByte_p(command, &config.AcBackupPoint, STR_AC_BACKUP_POINT, offset)) { }
        else if (parseConfigValByte_p(command, &config.AcRecoveredPoint, STR_AC_RECOVERED_POINT, offset)) { }
        else if (parseConfigValByte_p(command, &config.BatteryCapacityAh, STR_BATTERY_CAPACITY_AH, offset)) 
            resetSoc();
        else if (parseConfigValFloat_p(command, &config.BatteryEndingAmps, STR_BATTERY_ENDING_AMPS, offset)) { }
        else if (parseConfigValFloat_p(command, &config.BatteryAbsorbVoltage, STR_BATTERY_ABSORB_VOLTAGE, offset)) { }
        else
            fail = true;
    }

    else if (startsWith_p(command, STR_CONFIG, offset))
        printConfig(Serial, config, state);

    else if (startsWith_p(command, STR_VERSION, offset))
    {
        printCommandHeader_p(Serial, state, STR_VERSION);
        Serial.println(VERSION);
    }

    else if (startsWith_p(command, STR_SAVE, offset))
        writeConfig();

    else if (startsWith_p(command, STR_PAUSE, offset))
        state.EnableReadings = false;

    else if (startsWith_p(command, STR_RESUME, offset))
        state.EnableReadings = true;

    else if (startsWith_p(command, STR_PING, offset)) { }
    else if (startsWith_p(command, STR_PC, offset))
    {
        offset += strlen_P(STR_PC) + 1;

        if (startsWith_p(command, STR_ON, offset)) { }
        else if (startsWith_p(command, STR_OFF, offset))
            setPcDisconnected();
            
        else
            fail = true;
    }
    else if (startsWith_p(command, STR_DEHUM, offset))
    {
        offset += strlen_P(STR_DEHUM) + 1;

        if (startsWith_p(command, STR_ON, offset))
            setHumidityControl(true);
        else if (startsWith_p(command, STR_OFF, offset))
             setHumidityControl(false);
        else
            fail = true;
    }
    else if (startsWith_p(command, STR_RESET, offset))
    {
        offset += strlen_P(STR_RESET) + 1;

        if (startsWith_p(command, STR_SOC, offset)) 
            resetSoc();
        else if (startsWith_p(command, STR_CONFIG, offset)) 
            readConfig();
        else if (startsWith_p(command, STR_EEPROM, offset)) 
            writeDefaultConfig();
        else if (startsWith_p(command, STR_LOG, offset)) 
        { 
            bool needToReopen = (logFile == true);

            if (needToReopen)
                closeLogFile(true);

            SD.remove(logFileName);

            if (needToReopen)
                openLogFile(true, false);
        }
        else
            fail = true;
    }

    else
        fail = true;

    
    printCommandHeader_p(Serial, state, STR_CMD);
    printWithPipe(Serial, command);

    if (fail)
        writeLine_p(Serial, STR_FAIL);
    else
        writeLine_p(Serial, STR_OK);
}

/**
 * @brief Parse an incoming serial config floating point value
 */
boolean parseConfigValFloat_p(const char *command, float *destination, const char *itemName, uint8_t offset)
{
    if (!startsWith_p(command, itemName, offset))
        return false;

    offset += strlen_P(itemName) + 1;
    *destination = atof(command + offset);

    printConfigEntry_p(Serial, state, itemName, *destination);

    return true;
}

/**
 * @brief Parse an incoming serial config byte value
 */
boolean parseConfigValByte_p(const char *command, uint8_t *destination, const char *itemName, uint8_t offset)
{
    if (!startsWith_p(command, itemName, offset))
        return false;

    offset += strlen_P(itemName) + 1;
    *destination = atoi(command + offset);

    printConfigEntry_p(Serial, state, itemName, (uint32_t)*destination);

    return true;
}

/**
 * @brief Parse an incoming serial config short value
 */
boolean parseConfigValShort_p(const char *command, int8_t *destination, const char *itemName, uint8_t offset)
{
    if (!startsWith_p(command, itemName, offset))
        return false;

    offset += strlen_P(itemName) + 1;
    *destination = (int8_t)atoi(command + offset);

    printConfigEntry_p(Serial, state, itemName, (uint32_t)*destination);

    return true;
}

/**
 * @brief Parse an incoming serial config floating point value
 */
boolean parseConfigValInt_p(const char *command, uint16_t *destination, const char *itemName, uint8_t offset)
{
    if (!startsWith_p(command, itemName, offset))
        return false;

    offset += strlen_P(itemName) + 1;
    *destination = (uint16_t)atol(command + offset);

    printConfigEntry_p(Serial, state, itemName, (uint32_t)*destination);

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
    writeLine_p(Serial, STR_WRITE_DEFAULT_CONFIG);
    EEPROM.put(0, getDefaultConfig());

    readConfig();
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
    // state.UptimeSeconds = millis() / 1000;
    state.UptimeSeconds += 1;
    state.DehumCurrentStateSeconds += 1;
    state.BatteryCurrentStateSeconds += 1;
    state.LastPingSeconds += 1;

    // // if we are at the top of the minute, increment the minute counter
    // if (state.CurrentDtm->second() == 0)
    //     minutesSinceMidnight += 1;

    // // perform the midnight tick
    // //
    // // 1350 minutes is 22.5 hours, this is to ensure that we only roll the midnight clock once per day
    // // and we chose 22.5 hours just to make sure that, in case minutes may be missed, that there is 
    // // plenty of wiggle room for error
    // if (state.CurrentDtm->hour() == 0 && state.CurrentDtm->minute() == 0 && minutesSinceMidnight > 1350)
    // {
    //     Serial.println(F("midnight!"));

    //     // lets roll the log file at midnight
    //     rollLogFile();

    //     minutesSinceMidnight = 0;
    // }
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

#pragma region SD Card Logging Support
//========================================================================

/**
 * @brief Get the print object that is the target for log output. 
 *
 * Will either be the Serial connection, or, if the PC is disconnected, the
 * SD card log file
 */
Print &getPrintTarget()
{
    if (!state.PcConnected && logFile)
        return logFile;
    else
        return Serial;
}

/**
 * @brief Instruct the controller to begin dumping the log file back to the PC
 */
void startLogDump()
{
    setPcConnected();

    openLogFile(false, false);

    logFile.seek(0);

    doReadLog = true;
}

/**
 *  @brief Read the log file out to the serial connection
 * 
 * This is designed to be called on each loop in a way that it will
 * dump a portion of the log, then after a specified amount of time
 * has elapsed, it'll abort so that the program can continue to 
 * read sensors and accept PC input.
 */
void readLog()
{
    if (!doReadLog)
        return;

    if (logFile)
    {
        bool needPrefix = true;
        int character = -1;

        // we track the start time so that we don't take too many CPU cycles
        uint32_t dumpStartTime = millis();
        bool aborted = false;

        // while (logFile.available()) 
        while ((character = logFile.read()) != -1)
        {
            if (needPrefix == true)
            {
                write_p(Serial, STR_HIST);
                needPrefix = false;
            }

            Serial.write(character);

            if (character == '\n')
            {
                // at the end of each line, we check to see if we have exceeded the maximum
                // allowed dump duration
                if ((uint32_t)(millis() - dumpStartTime) > MAX_DUMP_MS_PER_CYCLE)
                {
                    aborted = true;
                    break;
                }
                else
                    needPrefix = true;
            }
        }

        if (!aborted)
        {
            doReadLog = false;
            closeLogFile(true);

            SD.remove(logFileName);
        }
    }
}

/**
 * @brief Switch the controller into "PC Disconnected" mode. This enabled SD logging.
 */
void setPcDisconnected()
{
    state.PcConnected = false;

    if (doReadLog)
    {
        doReadLog = false;
        closeLogFile(false);
    }

    openLogFile(true, false);
}

/**
 * @brief Switch the controller into "PC connected" mode. This disables SD logging
 */
void setPcConnected()
{
    state.LastPingSeconds = 0;
    state.PcConnected = true;

    closeLogFile(true);
}

/**
 * @brief Clush the buffer to the SD card. Needs to be called any time text
 * is written to the SD card logfile
 */
void flushLog()
{
    if (logFile && !doReadLog && state.PcConnected == false)
        logFile.flush();
}

/**
 * @brief Open the log file on the SD card for reading or writing
 */
void openLogFile(bool log, bool overwrite)
{
    if (logFile)
        return;
    
    if (log == true)
    {
        printCommandHeader(Serial, state);
        writeLine_p(Serial, STR_OPEN);
    }

    // remove the existing file if we have overwrite enabled
    if (overwrite)
        SD.remove(logFileName);

    logFile = SD.open(logFileName, FILE_WRITE);

    if (!logFile)
    {
        writeLine_p(Serial, STR_SD_FAILED);
        panic();
    }

    if (log == true)
    {
        printCommandHeader(logFile, state);
        writeLine_p(logFile, STR_OPEN);
    }
}

void setLogFileName(bool useSocLog)
{
    memset(logFileName, '\0', sizeof(logFileName));

    if (useSocLog)
        strcpy_P(logFileName, STR_SOC_FILE_NAME);
    else
        strcpy_P(logFileName, STR_LOG_FILE_NAME);
}

/**
 * @brief Close the log file
 */
void closeLogFile(bool log)
{
    if (logFile)
    {
        if (log == true)
        {
            printCommandHeader(Serial, state);
            writeLine_p(Serial, STR_CLOSE);

            printCommandHeader(logFile, state);
            writeLine_p(logFile, STR_CLOSE);
        }

        logFile.close();

        logFile = File();
    }
}

//========================================================================
#pragma endregion

byte previousLedState;
byte doPulse = 0;
byte currentPulseCount = 0;
byte targetPulseCount = 0;
byte pulseDelay = 100;
uint32_t lastPulseTime = 0;

void setPulse(byte count, byte dly)
{
    doPulse = 1;
    targetPulseCount = count;
    currentPulseCount = 0;
    pulseDelay = dly;
    previousLedState = state.DehumEnabled;
    lastPulseTime = millis();
}

void pulseLed()
{
    if (doPulse == 0)
        return;

    if ((uint32_t)(millis() - lastPulseTime) > pulseDelay)
    {
        previousLedState = !previousLedState;
        digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, previousLedState);
        lastPulseTime = millis();

        currentPulseCount++;

        if (targetPulseCount == currentPulseCount)
        {
            doPulse = 0;
            digitalWrite(PIN_DEHUMIDIFIER_ENABLED_LED, state.DehumEnabled);
        }
    }
}

void resetSoc()
{
    state.BatterySoc = 100;
    state.BatteryCapacityAh = config.BatteryCapacityAh;
    state.BatteryCapacityMicroAH = state.BatteryCapacityAh * 10000000UL;
}