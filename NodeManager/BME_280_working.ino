/**
   
*/



// Enable debug prints to serial monitor
#define MY_DEBUG

#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
#define MY_RADIO_NRF24

// ----------------------------------------------------------------------------
// Setup libraries
// ----------------------------------------------------------------------------
#include <MySensors.h>
#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <BH1750.h>

// ----------------------------------------------------------------------------
// Child sensor ids
// ----------------------------------------------------------------------------
#define BARO_CHILD 0
#define TEMP_CHILD 1
#define HUM_CHILD 2
#define CHILD_ID_LIGHT 3


// ----------------------------------------------------------------------------
// Sensor operation modes
// ----------------------------------------------------------------------------

BME280I2C bme;
BH1750 lightSensor;

// ----------------------------------------------------------------------------
// Light sensor variable
// ----------------------------------------------------------------------------
uint16_t lastlux;

// ----------------------------------------------------------------------------
// Define send thresholds
// ----------------------------------------------------------------------------
#define FORCE_TRANSMIT_INTERVAL 30
#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5
#define PRES_TRANSMIT_THRESHOLD 0.1
#define LIGHT_TRANSMIT_THRESHOLD 10
#define BATT_TRANSMIT_THRESHOLD 1

// ----------------------------------------------------------------------------
// BME280 variables and functions
// ----------------------------------------------------------------------------
// Adapt this constant: set it to the altitude above sealevel at your home location.
const float ALTITUDE = 10; // meters above sea level.

// Set this to true if you want to send values altough the values did not change.
// This is only recommended when not running on batteries.
const bool SEND_ALWAYS = false;

//////////////////////////////////////////////////////////
// You should not need to edit anything after this line //
//////////////////////////////////////////////////////////

#define SN "BaroHumTemp"        // Name of the sketch
#define SV "1.2"                // Version

// Constant for the world wide average pressure
const float SEALEVEL_PRESSURE = 1013.25;

// Sleep time between reads (in ms). Do not change this value as the forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000;
unsigned long previousMillis = 0;

const char *weatherStrings[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,       // Stable weather
  SUNNY = 1,        // Slowly rising HP stable good weather
  CLOUDY = 2,       // Slowly falling Low Pressure System, stable rainy weather
  UNSTABLE = 3,     // Quickly rising HP, not stable weather
  THUNDERSTORM = 4, // Quickly falling LP, Thunderstorm, not stable
  UNKNOWN = 5       // Unknown, more time needed
};

const char *situationStrings[] = { "very low", "low", "normal", "high", "very high" };
enum WEATHER_SITUATION
{
  VERY_LOW_PRESSURE = 0,   // p > -7.5hPa
  LOW_PRESSURE = 1,        // p > -2.5hPa
  NORMAL_PRESSURE = 2,     // p < +/-2.5hPa
  HIGH_PRESSURE = 3,       // p > +2.5hPa
  VERY_HIGH_PRESSURE = 4,  // p > +7.5hPa
};

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
int lastForecast = -1;
int lastSituation = NORMAL_PRESSURE;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from hPa to kPa in the forecast algorithm
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
MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage humMsg(HUM_CHILD, V_HUM);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);
MyMessage situationMsg(BARO_CHILD, V_VAR1);
MyMessage forecastMsg2(BARO_CHILD, V_VAR2);
MyMessage msg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

void initPressureSensor() {
  
  Wire.begin();
  Serial.begin(MY_BAUD_RATE);
  while(!Serial) {} // Wait
  while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
}


int getWeatherSituation(float pressure)
{
  int situation = NORMAL_PRESSURE;

  float delta = pressure - SEALEVEL_PRESSURE;
  if (delta > 7.5)
  {
    situation = VERY_HIGH_PRESSURE;
  }
  else if (delta > 2.5)
  {
    situation = HIGH_PRESSURE;
  }
  else if (delta < -7.5)
  {
    situation = VERY_LOW_PRESSURE;
  }
  else if (delta < -2.5)
  {
    situation = LOW_PRESSURE;
  }
  else
  {
    situation = NORMAL_PRESSURE;
  }

  return situation;
}


float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }

  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  // Uncomment when debugging
  Serial.print(F("### 5min-Average:"));
  Serial.print(lastPressureSamplesAverage);
  Serial.println(F(" hPa"));

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
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // Uncomment when dubugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weatherStrings[forecast]);

  return forecast;
}


bool updatePressureSensor()
{
  bool changed = false;
  float temperature = bme.temp();
  float humidity = bme.hum();
  float pressure_local = bme.pres() / 100.0; // Read atmospheric pressure at local altitude
  float pressure = ( pressure_local / pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255)); // Local pressure adjusted to sea level pressure using user altitude

  if (!metric)
  {
    // Convert to fahrenheit
    temperature = temperature * 9.0 / 5.0 + 32.0;
  }

  int forecast = sample(pressure);
  int situation = getWeatherSituation(pressure);

  if (SEND_ALWAYS || (temperature != lastTemp))
  {
    changed = true;
    lastTemp = temperature;
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(metric ? F(" *C") : F(" *F"));
    if (!send(tempMsg.set(lastTemp, 1)))
    {
      lastTemp = -1.0;
    }
  }

  if (SEND_ALWAYS || (humidity != lastHum))
  {
    lastHum = humidity;
    changed = true;
    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    if (!send(humMsg.set(lastHum, 1)))
    {
      lastHum = -1.0;
    }
  }

  if (SEND_ALWAYS || (pressure != lastPressure))
  {
    changed = true;
    lastPressure = pressure;
    Serial.print(F("Sea level Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));
    if (!send(pressureMsg.set(lastPressure, 1)))
    {
      lastPressure = -1.0;
    }
  }

  if (SEND_ALWAYS || (forecast != lastForecast))
  {
    changed = true;
    lastForecast = forecast;
    Serial.print(F("Forecast = "));
    Serial.println(weatherStrings[forecast]);
    if (send(forecastMsg.set(weatherStrings[lastForecast])))
    {
      if (!send(forecastMsg2.set(lastForecast)))
      {
      }
    }
    else
    {
      lastForecast = -1.0;
    }
  }

  if (SEND_ALWAYS || (situation != lastSituation))
  {
    changed = true;
    lastSituation = situation;
    Serial.print(F("Situation = "));
    Serial.println(situationStrings[situation]);
    if (!send(situationMsg.set(lastSituation)))
    {
      lastSituation = -1.0;
    }
  }

  return changed;
}


void updateLightLevel()      
{     
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  Serial.println(lux);
  if (lux != lastlux) {
      send(msg.set(lux));
      lastlux = lux;
  }

  sleep(SLEEP_TIME);
}

void setup() {
  // Setup locally attached sensors
  initPressureSensor();
  lightSensor.begin();
  metric = getControllerConfig().isMetric;  // was getConfig().isMetric; before MySensors v2.1.1
}


void presentation() {
  // Present locally attached sensors

  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SN, SV);

  // Register sensors to gw (they will be created as child devices)
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
}


void loop() {
  // Send locally attached sensor data here
  updatePressureSensor();
  updateLightLevel();     

  wait(SLEEP_TIME);   // do not use sleep() or delay(), it would prevent required TCP/IP and MySensor operations!
}
