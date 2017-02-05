#include <EEPROM.h>
// Firmware for telescope power monitor

// TODO:
// blink LED on battery low voltage (configurable)
// blink LED on battery high current draw (configurable)
// attempt to implement capacity tracking ??

// Output Format:
// <UptimeSeconds>|<AvgVoltage>|<MinVoltage>|<MaxVoltage>|<AvgAmperage>|<MinAmperage>|<MaxAmperage>

const String VERSION = "1.0";

struct ConfigObject
{
  boolean Defined;
  int AverageReadingCount; // 3 - 128
  float UpdateFrequency; // 1 - 20
  float WriteInterval; // 0.5 - 15, interval 0.5
  float VoltageCalibration; // text input // 0.0585
  int R1Actual; // text input
  int R2Actual; // text input
  int AmpDigitalOffset; // -5 to +5, interval 1
} defaultConfig = {
  true,
  30,
  10,
  1.0,
  0.0,
  10000,
  22000,
  0
};

int mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
int ACSoffset = 2500;

long uptimeSeconds;

float* amps = 0;
float* volts = 0;

long lastWriteTime = 0;

String inputString = "";
bool stringComplete = false;

bool enableReadings = true;

ConfigObject config;

// one time setup
void setup() 
{
  Serial.begin(9600);  
  
  ReadConfig();

  if (config.Defined == false)
  {
    WriteDefaultConfig();

    ReadConfig();
  }

  uptimeSeconds = 0;

  allocateArrays();
}

// mian loop
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
    pushReading(volts, getVoltage(A0));
    pushReading(amps, getAmperage(A1));
  
    // we only want to send the current values periodically, even though we refresh internally multiple times per second
    if (((millis() - lastWriteTime) >= (config.WriteInterval * 1000)) || lastWriteTime == 0)
      writeStatus();
    
    delay(1000 / config.UpdateFrequency);
  }
}

void allocateArrays()
{
  amps = new float[config.AverageReadingCount];
  volts = new float[config.AverageReadingCount];

  initializeArray(amps);
  initializeArray(volts); 
}

void reallocateArrays()
{
  if (amps != 0)
    delete [] amps;

  if (volts != 0)
    delete [] volts;

  allocateArrays();
}

void ReadConfig()
{
  // load the config from the EEPROM
  EEPROM.get(0, config);
}

void WriteDefaultConfig()
{
  Serial.println("Writing default config to EEPROM");
  EEPROM.put(0, defaultConfig);
}


void initializeArray(float* readingArray)
{  
  for (int i = 0; i < config.AverageReadingCount; i++)
    readingArray[i] = 0.0;
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

void handleSerialCommand(String* commandLine)
{
  if (commandLine->startsWith("GET"))
  {
    // strip the GET from the beginning
    *commandLine = commandLine->substring(4);

    if (commandLine->startsWith("Version"))
    {
      Serial.print("VERSION: ");
      Serial.println(VERSION);
    }
    
    else if (commandLine->startsWith("Config"))
    {
      Serial.print("CONFIG|");
      Serial.print("AverageReadingCount:");
      Serial.println(config.AverageReadingCount);

      Serial.print("CONFIG|");
      Serial.print("UpdateFrequency:");
      Serial.println(config.UpdateFrequency);

      Serial.print("CONFIG|");
      Serial.print("WriteInterval:");
      Serial.println(config.WriteInterval);

      Serial.print("CONFIG|");
      Serial.print("VoltageCalibration:");
      Serial.println(config.VoltageCalibration, 4);

      Serial.print("CONFIG|");
      Serial.print("R1Actual:");
      Serial.println(config.R1Actual);

      Serial.print("CONFIG|");
      Serial.print("R2Actual:");
      Serial.println(config.R2Actual);

      Serial.print("CONFIG|");
      Serial.print("AmpDigitalOffset:");
      Serial.println(config.AmpDigitalOffset);
    }
  }

  
  else if (commandLine->startsWith("SET"))
  {
    *commandLine = commandLine->substring(4);

    if (commandLine->startsWith("AverageReadingCount"))
    {
      *commandLine = commandLine->substring(20);
      config.AverageReadingCount = commandLine->toInt();
      
      Serial.print("CONFIG|AverageReadingCount:");
      Serial.println(config.AverageReadingCount);

      reallocateArrays();
    }

    else if (commandLine->startsWith("UpdateFrequency"))
    {
      *commandLine = commandLine->substring(16);
      config.UpdateFrequency = commandLine->toFloat();
      
      Serial.print("CONFIG|UpdateFrequency:");
      Serial.println(config.UpdateFrequency);
    }
    
    else if (commandLine->startsWith("WriteInterval"))
    {
      *commandLine = commandLine->substring(14);
      config.WriteInterval = commandLine->toFloat();
      
      Serial.print("CONFIG|WriteInterval:");
      Serial.println(config.WriteInterval);
    }
    
    else if (commandLine->startsWith("VoltageCalibration"))
    {
      *commandLine = commandLine->substring(19);
      config.VoltageCalibration = commandLine->toFloat();
      
      Serial.print("CONFIG|VoltageCalibration:");
      Serial.println(config.VoltageCalibration, 4);
    }
    
    else if (commandLine->startsWith("R1Actual"))
    {
      *commandLine = commandLine->substring(9);
      config.R1Actual = commandLine->toInt();
      
      Serial.print("CONFIG|R1Actual:");
      Serial.println(config.R1Actual);
    }

    else if (commandLine->startsWith("R2Actual"))
    {
      *commandLine = commandLine->substring(9);
      config.R2Actual = commandLine->toInt();
      
      Serial.print("CONFIG|R2Actual:");
      Serial.println(config.R2Actual);
    }

    else if (commandLine->startsWith("AmpDigitalOffset"))
    {
      *commandLine = commandLine->substring(17);
      config.AmpDigitalOffset = commandLine->toInt();
      
      Serial.print("CONFIG|AmpDigitalOffset:");
      Serial.println(config.AmpDigitalOffset);
    }
  }

  
  else if (commandLine->startsWith("SAVE"))
  {
    EEPROM.put(0, config); 
    Serial.println("EEPROM Written");
  }

  
  else if (commandLine->startsWith("CLEAR"))
  {
    WriteDefaultConfig();
    ReadConfig();
  }

  else if (commandLine->startsWith("PAUSE"))
  {
    enableReadings = false;
  }

  else if (commandLine->startsWith("RESUME"))
  {
    enableReadings = true;
  }
}

void writeStatus()
{
  uptimeSeconds = millis() / 1000;
    
  Serial.print(uptimeSeconds);
  Serial.print("|");
  
  Serial.print(getAvgReading(volts));
  Serial.print("|");
  
  Serial.print(getMinReading(volts));
  Serial.print("|");
  
  Serial.print(getMaxReading(volts));
  Serial.print("|");
  
  Serial.print(getAvgReading(amps));
  Serial.print("|");
  
  Serial.print(getMinReading(amps));
  Serial.print("|");
  
  Serial.print(getMaxReading(amps));
  Serial.println();

  lastWriteTime = millis();
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

