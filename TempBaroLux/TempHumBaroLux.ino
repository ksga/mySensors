/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 2.0 - Kenneth Andersen
 * 
 * DESCRIPTION
 * Pressure sensor/LUX example using BMP085 module and BH1750
 * http://www.mysensors.org/build/pressure
 *
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
#include <BH1750.h>
#include <RunningAverage.h>

// BME280 libraries and variables
// Bosch BME280 Embedded Adventures MOD-1022 weather multi-sensor Arduino code
// Written originally by Embedded Adventures
// https://github.com/embeddedadventures/BME280
#include <BME280_MOD-1022.h>

#define AVERAGES 2

#define CHILD_ID_BARO 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_HUM 2
#define CHILD_ID_LIGHT 3

#define R1 687000
#define R2 222000
#define VMIN 3.30
#define VMAX 4.18
#define ADC_PRECISION 1023
#define VREF 1.107

#define HUMI_TRANSMIT_THRESHOLD 1
#define TEMP_TRANSMIT_THRESHOLD 0.1
#define LUX_TRANSMIT_THRESHOLD 100

#define CONVERSION_FACTOR (1.0/10.0)

#define FORCE_TRANSMIT_INTERVAL 10

BH1750 lightSensor;

const float ALTITUDE = 10; // <-- adapt this value to your location's altitude (in m). Use your smartphone GPS to get an accurate value!

// Sleep time between reads (in ms). Do not change this value as the forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000; 

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];


// this CONVERSION_FACTOR is used to convert from Pa to kPa in the forecast algorithm
// get kPa/h by dividing hPa by 10 
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;
boolean metric;
MyMessage tempMsg(CHILD_ID_TEMP, V_TEMP);
MyMessage humMsg(CHILD_ID_HUM, V_HUM);
MyMessage pressureMsg(CHILD_ID_BARO, V_PRESSURE);
MyMessage forecastMsg(CHILD_ID_BARO, V_FORECAST);
MyMessage msg(CHILD_ID_LIGHT, V_LEVEL);

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}


// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}

// Global settings
int measureCount = 0;

uint16_t lastlux;
boolean isMetric = true;
boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = -100;
int lastHumidity = -100;

int oldBatteryPcnt = 0;
int BATTERY_SENSE_PIN = A0;

RunningAverage raHum(AVERAGES);

void setup() {
  metric = getConfig().isMetric;
  Wire.begin(); // Wire.begin(sda, scl)
  lightSensor.begin();
  raHum.clear();
  analogReference(INTERNAL);
  pinMode(BATTERY_SENSE_PIN, INPUT);
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BME280/BH1750 Sensor", "2.0");

  // Register sensors to gw (they will be created as child devices)
  present(CHILD_ID_BARO, S_BARO);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
}

void loop() {

  int batteryPcnt = getBatteryPercentage();
  if (oldBatteryPcnt != batteryPcnt) {
  // Power up radio after sleep
  sendBatteryLevel(batteryPcnt);
  oldBatteryPcnt = batteryPcnt;
  }  
  
  measureCount ++;
  bool forceTransmit = false;
  transmission_occured = false;


  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }

  sendTempHumBaroMeasurements(forceTransmit);

  sendLightLevel(forceTransmit);
//  sleep(MEASURE_INTERVAL);
  sleep(SLEEP_TIME);
}

int getBatteryPercentage() {

delay(500);
// read analog pin value
int inputValue = analogRead(BATTERY_SENSE_PIN);

// calculate the max possible value and therefore the range and steps
float voltageDividerFactor = (R1 + R2) / R2;
float maxValue = voltageDividerFactor * VREF;
float voltsPerBit = maxValue / ADC_PRECISION;

float batteryVoltage = voltsPerBit * inputValue;
Serial.print(batteryVoltage);
float batteryPercentage = ((batteryVoltage-VMIN)/(VMAX-VMIN))*100;
//int batteryPercentage = map(batteryVoltage, 0, maxValue, 0, 100);

return batteryPercentage;
}

// BME280 temp hum baro
void sendTempHumBaroMeasurements() {
  
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();

  /* After taking the measurement the chip goes back to sleep, use when battery powered.
  // Oversampling settings (os1x, os2x, os4x, os8x or os16x).
  BME280.writeFilterCoefficient(fc_16);       // IIR Filter coefficient, higher numbers avoid sudden changes to be accounted for (such as slamming a door)
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os8x);  // temperature x8
  BME280.writeOversamplingHumidity(os8x);     // humidity x8
  BME280.writeMode(smForced);                 // Forced sample.  After taking the measurement the chip goes back to sleep
  */

  // Normal mode for regular automatic samples
  BME280.writeStandbyTime(tsb_0p5ms);         // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);       // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os8x);  // temperature x8
  BME280.writeOversamplingHumidity(os8x);     // humidity x8
  
  BME280.writeMode(smNormal);
  
  while (1) {
    // Just to be sure, wait until sensor is done mesuring  
    while (BME280.isMeasuring()) {
  }
  
  // Read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();

  float temperature = BME280.getTemperatureMostAccurate();                    // must get temp first
  float humidity = BME280.getHumidityMostAccurate();
  float pressure_local = BME280.getPressureMostAccurate();                    // Get pressure at current location
  float pressure = pressure_local/pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255); // Adjust to sea level pressure using user altitude
  int forecast = sample(pressure);
  
  if (!metric) 
  {
    // Convert to fahrenheit
    temperature = temperature * 9.0 / 5.0 + 32.0;
  }

  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(metric ? " °C" : " °F");
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("Forecast = ");
  Serial.println(weather[forecast]);
  Serial.println();


  if (temperature != lastTemp) 
  {
    send(tempMsg.set(temperature, 1));
    lastTemp = temperature;
  }


  if (humidity != lastHum) 
  {
    send(humMsg.set(humidity, 1));
    lastHum = humidity;
  }

  if (pressure != lastPressure) 
  {
    send(pressureMsg.set(pressure, 2));
    lastPressure = pressure;
  }

  if (forecast != lastForecast)
  {
    send(forecastMsg.set(weather[forecast]));
    lastForecast = forecast;
  }
  
//  sleep(SLEEP_TIME);
 } 
}
  void sendLightLevel(bool force)      
{     
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  Serial.println(lux);
  if (lux != lastlux) {
      send(msg.set(lux));
      lastlux = lux;
  }

}
