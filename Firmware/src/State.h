#ifndef _STATE_H
#define _STATE_H

#include "lib/RTC/RTClib.h"

#include <Arduino.h>

/**
 * @brief Object used to store the current state of the controller
 */
struct State
{
    /**
     * @brief If true, the firmware will read from all sensors on board
     */
    boolean EnableReadings;

    /** 
     * @brief Milliseconds of last time status was written to serial port
     */
    uint32_t LastWriteTime;

    /**
     * @brief Last voltage/amperage sensor read time
     */
    uint32_t LastReadTime;

    /**
     * @brief Milliseconds of last time the system clock and status was updated
     */
    uint32_t LastTick;

    /**
     * @brief Milliseconds of last time DHT (temp/hum) sensor was probed
     */
    uint32_t LastDhtReadTime;

    /**
     * @brief Number of milliseconds of time the humidity was last checked
     */
    uint32_t LastHumidityCheckTime;

    /**
     * @brief Number of milliseconds of time the battery voltage was last checked
     */
    uint32_t LastBatteryCheckTime;

    /**
     * @brief Most recent DTM as read from the RTC
     */
    DateTime* CurrentDtm;

    /**
     * @brief Non-exact number of seconds the telescope power monitor has been powered up
     * 
     * This relies on the internal clock of the ATMega chip, therfore is inexact. 
     * Additionally, this will overflow after roughly 68 years ;)
     */
    uint32_t UptimeSeconds;

    /**
     * @brief Most recent temperature reading
     */
    float Temperature;

    /**
     * @brief Most recent humidity reading
     */
    float Humidity;

    /**
     * @brief How many seconds has the dehumidifier been in the current state
     */
    uint32_t DehumCurrentStateSeconds;

    /**
     * @brief How many seconds has the battery been in the current state
     */
    uint32_t BatteryCurrentStateSeconds;

    /**
     * @brief How many seconds have elapsed since the last PING from the PC
     */
    uint32_t LastPingSeconds;

    /**
     * @brief Flag indicating whether automatic humidity control is enabled
     */
    boolean DehumEnabled;

    /**
     * @brief Flag indicating whether the Dehumidifier output is turned on
     */
    boolean DehumOutState;

    /**
     * @brief Flag indicating whether the telescope output is turned on
     */
    boolean TelescopeOutState;

    /**
     * @brief Flag indicating whether the AUX1 output is turned on
     */
    boolean Aux1OutState;

    /**
     * @brief Flag indicating whether the AC power supply is connected
     */
    boolean AcInState;

    /**
     * @brief Current system voltage
     */
    float Volt;

    /**
     * @brief Current amperage coming from the battery. A positive value indicates 
     * amperage flowing INTO the battery. A negative value indicates amperage flowing 
     * OUT of the battery
     */
    float BatteryAmp;

    // TODO: prevent positive values?
    /**
     * @brief Current amperage being drawn by the load. Positive indicates amperage 
     * flowing TO the load. Negative indicates amperage flowing FROM the load (should 
     * no really happen, but won't hurt anything if it does)
     */
    float LoadAmp;

    // TODO: prevent negative values?
    /**
     * @brief Current amperage coming from solar power supply. Positive indicates amperage
     * coming FROM the power supply, negative indicates amperage flowing TO the power supply 
     * (should never happen)
     */
    float SolarAmp;

    // TODO: prevent negative values?
    /**
     * @brief Current amperage coming from AC PSU. Positive indicates current flowing FROM the power
     * supply, negative indicates amperage flowing TO the power supply (should never happen)
     */
    float AcAmp;

    /**
     * @brief Flag indicating whether the PC is currently connected
     */
    bool PcConnected;

    /**
     * @brief Current battery state of charge.
     * 
     * This is stored to the EEPROM every 10%. This is done to minimize stress on the EEPROM
     * while still allowing the SOC to be remembered (mostly) after a power loss.
     */
    float BatterySoc;

    /**
     * @brief How much capacity is remaining in the battery
     */
    float BatteryCapacityAh;

    /**
     * @brief How much capacity is remaining in the battery, in microamps.. this is 
     * for internal tracking and calculation purposes only
     */
    uint32_t BatteryCapacityMicroAH;
};

#endif // _STATE_H