#ifndef _STRINGS_H
#define _STRINGS_H

#include <Arduino.h>

const char str_AverageReadingCount[] PROGMEM = "AverageReadingCount";
const char str_UpdateFrequency[] PROGMEM = "UpdateFrequency";
const char str_WriteInterval[] PROGMEM = "WriteInterval";
const char str_VoltageCalibration[] PROGMEM = "VoltageCalibration";
const char str_R1Actual[] PROGMEM = "R1Actual";
const char str_R2Actual[] PROGMEM = "R2Actual";
const char str_AmpDigitalOffset1[] PROGMEM = "AmpDigitalOffset1";
const char str_AmpDigitalOffset2[] PROGMEM = "AmpDigitalOffset2";
const char str_AmpDigitalOffset3[] PROGMEM = "AmpDigitalOffset3";
const char str_TemperatureCalibration[] PROGMEM = "TemperatureCalibration";
const char str_HumidityCalibration[] PROGMEM = "HumidityCalibration";
const char str_TargetHumidity[] PROGMEM = "TargetHumidity";
const char str_HumidityHysterisis[] PROGMEM = "HumidityHysterisis";
const char str_AcBackupPoint[] PROGMEM = "AcBackupPoint";
const char str_Config[] PROGMEM = "CONFIG";
const char str_Version[] PROGMEM = "VERSION";
const char str_Save[] PROGMEM = "SAVE";
const char str_Clear[] PROGMEM = "CLEAR";
const char str_Pause[] PROGMEM = "PAUSE";
const char str_Resume[] PROGMEM = "RESUME";
const char str_Ok[] PROGMEM = "OK";
const char str_Fail[] PROGMEM = "FAIL";
const char str_Pwr[] PROGMEM = "PWR";
const char str_On[] PROGMEM = "ON";
const char str_Off[] PROGMEM = "OFF";
const char str_Telescope[] PROGMEM = "TELESCOPE";
const char str_Aux1[] PROGMEM = "AUX1";
const char str_Dehum[] PROGMEM = "DEHUM";
const char str_Ac[] PROGMEM = "AC";
const char str_Set[] PROGMEM = "SET";
const char str_Time[] PROGMEM = "TIME";
const char str_WriteDefaultConfig[] PROGMEM = "WRITE default config";
const char str_SdFailed[] PROGMEM = "SD failed";
const char str_RtcFail[] PROGMEM = "RTC FAIL";
const char str_RtcInit[] PROGMEM = "RTC Init";
const char str_Cmd[] PROGMEM = "CMD";
const char str_Stat[] PROGMEM = "STAT";
const char str_Env[] PROGMEM = "ENV";
const char str_Ping[] PROGMEM = "PING";


const char *const string_table[] PROGMEM = {
    str_AverageReadingCount,
    str_UpdateFrequency,
    str_WriteInterval,
    str_VoltageCalibration,
    str_R1Actual,
    str_R2Actual,
    str_AmpDigitalOffset1,
    str_AmpDigitalOffset2,
    str_AmpDigitalOffset3,
    str_TemperatureCalibration,
    str_HumidityCalibration,
    str_TargetHumidity,
    str_HumidityHysterisis,
    str_AcBackupPoint,
    str_Config,
    str_Version,
    str_Save,
    str_Clear,
    str_Pause,
    str_Resume,
    str_Ok,
    str_Fail,
    str_Pwr,
    str_On,
    str_Off,
    str_Telescope,
    str_Aux1,
    str_Dehum,
    str_Ac,
    str_Set,
    str_Time,
    str_WriteDefaultConfig,
    str_SdFailed,
    str_RtcFail,
    str_RtcInit,
    str_Cmd,
    str_Stat,
    str_Env,
    str_Ping
};

#define STR_AVERAGE_READING_COUNT string_table[0]
#define STR_UPDATE_FREQUENCY string_table[1]
#define STR_WRITE_INTERVAL string_table[2]
#define STR_VOLTAGE_CALIBRATION string_table[3]
#define STR_R1_ACTUAL string_table[4]
#define STR_R2_ACTUAL string_table[5]
#define STR_AMP_DIGITAL_OFFSET_1 string_table[6]
#define STR_AMP_DIGITAL_OFFSET_2 string_table[7]
#define STR_AMP_DIGITAL_OFFSET_3 string_table[8]
#define STR_TEMPERATURE_CALIBRATION string_table[9]
#define STR_HUMIDITY_CALIBRATION string_table[10]
#define STR_TARGET_HUMIDITY string_table[11]
#define STR_HUMIDITY_HYSTERISIS string_table[12]
#define STR_AC_BACKUP_POINT string_table[13]
#define STR_CONFIG string_table[14]
#define STR_VERSION string_table[15]
#define STR_SAVE string_table[16]
#define STR_CLEAR string_table[17]
#define STR_PAUSE string_table[18]
#define STR_RESUME string_table[19]
#define STR_OK string_table[20]
#define STR_FAIL string_table[21]
#define STR_PWR string_table[22]
#define STR_ON string_table[23]
#define STR_OFF string_table[24]
#define STR_TELESCOPE string_table[25]
#define STR_AUX1 string_table[26]
#define STR_DEHUM string_table[27]
#define STR_AC string_table[28]
#define STR_SET string_table[29]
#define STR_TIME string_table[30]
#define STR_WRITE_DEFAULT_CONFIG string_table[31]
#define STR_SD_FAILED string_table[32]
#define STR_RTC_FAIL string_table[33]
#define STR_RTC_INIT string_table[34]
#define STR_CMD string_table[35]
#define STR_STAT string_table[36]
#define STR_ENV string_table[37]
#define STR_PING string_table[38]

#endif // _STRINGS_H