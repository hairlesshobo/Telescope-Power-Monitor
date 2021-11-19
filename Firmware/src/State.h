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
     * @brief If true, the output to the telescope is enabled
     */
    boolean TelescopeOutputEnabled;

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

};

#endif // _STATE_H