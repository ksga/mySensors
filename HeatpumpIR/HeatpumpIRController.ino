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
 * Version 1.0 - Toni A - https://github.com/ToniA/arduino-heatpumpir
 * Version 1.1 - ksga added DS18B20
 * Version 1.2 - Added logic to only send temperature is changed and valid. Removed unused lines.
 *
 * DESCRIPTION
 * Heatpump controller
 */


 /*
  * This sketch is for the cabin project, to control a heatpump, running on Sensebender
  * https://www.mysensors.org/hardware/micro
  *
  * The sketch is based on the 'HeatpumpIRController' example of the MySensors project
  */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Channel 1
#define MY_RF24_CHANNEL  76

// Safe baud rate for a 3.3V device
#define MY_BAUD_RATE 9600

// HeatpumpIR libraries,
//#include <FujitsuHeatpumpIR.h>
#include <PanasonicCKPHeatpumpIR.h>
#include <PanasonicHeatpumpIR.h>
//#include <CarrierHeatpumpIR.h>
//#include <MideaHeatpumpIR.h>
//#include <MitsubishiHeatpumpIR.h>
//#include <SamsungHeatpumpIR.h>
//#include <SharpHeatpumpIR.h>
//#include <DaikinHeatpumpIR.h>

// DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Timer library, https://github.com/JChristensen/Timer
#include <Timer.h>

// MySensors libraries
#include <SPI.h>
#include <MySensors.h>

// Child ID's of this node
#define CHILD_IR     1
#define CHILD_TEXT   2
#define CHILD_SWITCH 3

// DS18B20 temperature sensor
#define CHILD_TEMP   4

// MySensors messages of this node
MyMessage irMsg(CHILD_IR, V_IR_RECEIVE);
MyMessage textMsg(CHILD_TEXT, V_TEXT);
MyMessage switchMsg(CHILD_SWITCH, V_LIGHT);

// DS18B20 temperature sensor
MyMessage temperatureMsg(CHILD_TEMP, V_TEMP);

// Array with all supported heatpump models
HeatpumpIR *heatpumpIR[] = { new PanasonicCKPHeatpumpIR(), // 0, keep this if you don't remove the timer for cancelling Panasonic CKP messages
                             //new PanasonicDKEHeatpumpIR(), // 1
                             //new PanasonicJKEHeatpumpIR(), // 2
                             //new PanasonicNKEHeatpumpIR(), // 3
                             new PanasonicLKEHeatpumpIR(), // 4
                             //new CarrierNQVHeatpumpIR(),   // 5
                             //new CarrierMCAHeatpumpIR(),   // 6
                             //new MideaHeatpumpIR(),        // 7
                             //new FujitsuHeatpumpIR(),      // 8
                             //new MitsubishiFDHeatpumpIR(), // 9
                             //new MitsubishiFEHeatpumpIR(), // A
                             //new SamsungHeatpumpIR(),      // B
                             //new SharpHeatpumpIR(),        // C
                             //new DaikinHeatpumpIR()        // D
                           };

uint8_t models = sizeof(heatpumpIR) / sizeof(HeatpumpIR*);

// IR led on PWM output-capable digital pin 3
IRSenderPWM irSender(3);

// Timer for sending temperature&humidity measurements & Panasonic CKP series timer cancellation commands
Timer timer;
int8_t tempMeasureTimer = 1;
int8_t panasonicCKPTimer = 1;

// DS18B20 temperature sensor
#define ONE_WIRE_BUS 4
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//DS18B20 address. Use sketch at https://arduino-info.wikispaces.com/Brick-Temperature-DS18B20#Test%20Sketch%20to%20read%20DS18B20%20addresses to find address of sensor.
DeviceAddress insideThermometer = { 0x28, 0x8C, 0x23, 0x04, 0x00, 0x00, 0x80, 0xBD };
float lastTemperature;

void setup()
{
  Serial.println(F("HeatpumpIR sensor starting up..."));
  Serial.print(F("Number of supported models: ")); Serial.println(models);

  // The TEXT sensor is not created in Domoticz before data is sent
  send(textMsg.setSensor(CHILD_TEXT).set("00000000"));

// DS18B20 temperature sensor
  tempMeasureTimer = timer.every(180000, sendTempMeasurements);

  sensors.begin();
  sensors.setResolution(insideThermometer, 11);
}


void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Heatpump TempSensor", "1.1");

  // Register the sensors to the MySensors Gateway
  present(CHILD_IR, S_IR, "IR sender");

// DS18B20 temperature sensor
  present(CHILD_TEMP,S_TEMP, "Temperature");

  present(CHILD_TEXT, S_INFO, "IR data");
  present(CHILD_SWITCH, S_LIGHT, "IR send");
}


void loop()
{
  timer.update();
}


// Handle incoming messages from the MySensors Gateway
void receive(const MyMessage &message) {

  const char *irData;

  // V_IR type message
  if (message.type==V_IR_SEND) {
    Serial.println(F("Received IR send command..."));
    irData = message.getString();
    Serial.print(F("Code: 0x"));
    Serial.println(irData);
    sendHeatpumpIRCommand(irData);
  }
  // -- Hacks for Domoticz start
  else if (message.type==V_LIGHT) {
    // When the button is pressed on Domoticz, request the value of the TEXT sensor
    Serial.println(F("Requesting IR code from Domoticz..."));
    request(CHILD_TEXT, V_TEXT, 0);
  } else if (message.type==V_TEXT) {
    // TEXT sensor value is received as a result of the previous step
    Serial.println(F("IR code received from Domoticz..."));
    Serial.print(F("Code: 0x"));

    irData = message.getString();
    Serial.println(irData);

    sendHeatpumpIRCommand(irData);

    // Set the Domoticz switch back to 'OFF' state
    send(switchMsg.setSensor(CHILD_SWITCH).set(0));
  }
  // -- Hacks for Domoticz end
}


// Decode the IR command and send the IR command to the heatpump
void sendHeatpumpIRCommand(const char *irCommandString) {

  // irCommandString is an 8-digit hex digit
  long irCommand = 0;
  int sscanfStatus = sscanf(irCommandString, "%lx", &irCommand);

  if (sscanfStatus == 1) {
    Serial.print(F("IR code conversion OK: 0x"));
    Serial.println(irCommand, HEX);
  } else {
    Serial.println(F("Failed to convert IR hex code to number"));
  }

/*
The heatpump command is packed into a 32-bit hex number, see
libraries\HeatpumpIR\HeatpumpIR.h for the constants

12345678
  3 MODEL
   4 POWER
    5 OPERATING MODE
     6 FAN SPEED
      78 TEMPERATURE IN HEX

00213416 (as an example of a valid code)
  2 = PanasonicJKE
   1 = Power ON
    3 = COOL
     4 = FAN 4
      16 = Temperature 22 degrees (0x16 = 22)
 */

  byte model = (irCommand & 0x00F00000) >> 20;
  byte power = (irCommand & 0x00010000) >> 16;
  byte mode  = (irCommand & 0x0000F000) >> 12;
  byte fan   = (irCommand & 0x00000F00) >> 8;
  byte temp  = (irCommand & 0x000000FF);

  const char* buf;
  Serial.print(F("Model: "));

  buf = heatpumpIR[model]->model();
  // 'model' is a PROGMEM pointer, so need to write a byte at a time
  while (char modelChar = pgm_read_byte(buf++))
  {
    Serial.print(modelChar);
  }
  Serial.println();

  Serial.print(F("Model #: ")); Serial.println(model);
  Serial.print(F("Power: ")); Serial.println(power);
  Serial.print(F("Mode: ")); Serial.println(mode);
  Serial.print(F("Fan: ")); Serial.println(fan);
  Serial.print(F("Temp: ")); Serial.println(temp);

  // Heatpump models start from 0, i.e. model number is always less than the number of different models
  if (model < models) {
    // This is a supported model

    // Cancel the timer on Panasonic CKP heatpump
    if (model == 0) {
      Serial.println(F("Cancelling timer on Panasonic CKP heatpump..."));
      timer.stop(panasonicCKPTimer);
    }

    Serial.println(F("All OK - sending IR command to heatpump..."));
    heatpumpIR[model]->send(irSender, power, mode, fan, temp, VDIR_UP, HDIR_AUTO);

    if (model == 0) {
      Serial.println(F("Scheduling timer cancellation on Panasonic CKP heatpump..."));
      panasonicCKPTimer = timer.after(120000, panasonicCancelTimer); // Called after 2 minutes
    }
  }
}


// Cancel the timer on the Panasonic CKP heatpump
void panasonicCancelTimer()
{
  PanasonicCKPHeatpumpIR *panasonicCKPHeatpumpIR = new PanasonicCKPHeatpumpIR();

  panasonicCKPHeatpumpIR->sendPanasonicCKPCancelTimer(irSender);
  Serial.println(F("The TIMER led should now be OFF"));
}

// DS18B20 temperature sensor
void sendTempMeasurements()
{
  sensors.requestTemperatures();

  float temperature = sensors.getTempC(insideThermometer);

  Serial.print(F("T: "));Serial.println(temperature);

      // Only send data if temperature has changed and no error
    #if COMPARE_TEMP == 1
    if (lastTemperature != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif

      // Send in the new temperature
      send(temperatureMsg.set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature=temperature;
    }
}
