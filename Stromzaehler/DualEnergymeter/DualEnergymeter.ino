/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Armin Kaiser
* Based on ReflectorLightBarrier sketch by Martin Kompf and Watermeter sketch V 1.0 by Ralph Bisschops which is based on original Watermeter sketch V 1.1 by Henrik Ekblad and GizMoCuz
* Copyright (C) 2017 Armin Kaiser
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* REVISION HISTORY
* Version 0.1 - Armin Kaiser (first alpha release)
*
* DESCRIPTION
* This version is for two house enegymeters!!!
* Use the TCRT5000 sensor to measure electrical power and energy of your house energymeter.
* You need to set the correct pulsefactor of your meter (pulses per kWh).
* The sensor starts by fetching current energy reading from gateway (VAR 1).
* Upper and lower threshold level can be modified over Serial Connection by typing "C" for command mode 
* and subsequent typing "S 'lower threshold' 'upper threshold'" followed from enter.
* Reports both power and energy back to gateway.
*
* DESCRIPTION
* This is an example of sensors using RS485 as transport layer
*
* If your Arduino board has additional serial ports
* you can use to connect the RS485 module.
* Otherwise, the transport uses AltSoftSerial to handle two serial
* links on one Arduino. Use the following pins for RS485 link
*
*  Board          Transmit  Receive   PWM Unusable
* -----          --------  -------   ------------
* Teensy 3.0 & 3.1  21        20         22
* Teensy 2.0         9        10       (none)
* Teensy++ 2.0      25         4       26, 27
* Arduino Uno        9         8         10
* Arduino Leonardo   5        13       (none)
* Arduino Mega      46        48       44, 45
* Wiring-S           5         6          4
* Sanguino          13        14         12 *
*
* Unfortunately millis() won't increment when the Arduino is in
* sleep mode. So we cannot make this sensor sleep if we also want
* to calculate/report flow.
*/
// Enable debug prints to serial monitor
//#include <MyConfig.h>
#define MY_DEBUG 

// RS485
// Enable RS485 transport layer
//#define MY_RS485

// Define this to enables DE-pin management on defined pin
//#define MY_RS485_DE_PIN 2

// Set RS485 baud rate to use
//#define MY_RS485_BAUD_RATE 9600

// Enable this if RS485 is connected to a hardware serial port
//#define MY_RS485_HWSERIAL Serial1

// Radio
// Enable and select radio type attached (Don't forget to disable RS485)
//#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <MySensors.h>

// Node and sketch information
#define SKETCH_VER            "0.1"            // Sketch version
#define SKETCH_NAME           "Dual Energy Meter" // Sketch name
#define CHILD_ID1             1             // Id of the sensor child one
#define CHILD_NAME1           "Energy Meter"    // Optional child sensor name one
#define CHILD_ID2             2             // Id of the sensor child two
#define CHILD_NAME1           "Energy Meter"    // Optional child sensor name two
#define NODE_REPEATER         false           // Set true if a repeater node (i.e. always turned on)
#define SLEEP_MODE            false           // powervalue can only be reported when sleep mode is false.
//#define MY_NODE_ID          101                 // Node ID if RS485 (Change NODE_ID to your needs)

// Sketch settings
#define PULSE_FACTOR          75          // Number of blinks/rpm per kWh of your meter (75 rotation/kWh)
#define MAX_WATT        11000           // Max watt value to report. This filters outliers. (heat pump  ~ 10kW) (11KWh ~825 U/h ~ 0,23 U/s)

// Input and output definitions
//#define DIGITAL_INPUT_SENSOR1  3          // The digital input you attached your sensor one.  (Only 2 and 3 generates interrupt!)
#define ANALOG_INPUT_SENSOR1   A0         // The analog input you you attached your sensor one. 
//#define DIGITAL_INPUT_SENSOR2  5          // The digital input you attached your sensor two.  (Only 2 and 3 generates interrupt!) (D2 is connected to RFM69 Radio only use with RS485!)
#define ANALOG_INPUT_SENSOR2   A1         // The analog input you you attached your sensor two. 

MyMessage wattMsg1(CHILD_ID1,V_WATT);
MyMessage wattMsg2(CHILD_ID2,V_WATT);
MyMessage kwhMsg1(CHILD_ID1,V_KWH);
MyMessage kwhMsg2(CHILD_ID2,V_KWH);
MyMessage lastCounterMsg1(CHILD_ID1, V_VAR1);
MyMessage lastCounterMsg2(CHILD_ID2, V_VAR1);
MyMessage highthresholdMsg1(CHILD_ID1,V_VAR3);
MyMessage highthresholdMsg2(CHILD_ID2,V_VAR3);
MyMessage lowthresholdMsg1(CHILD_ID1,V_VAR4);
MyMessage lowthresholdMsg2(CHILD_ID2,V_VAR4);

// Global vars
#define nEM 2                       // n = Number of Energymeters
unsigned long sendFrequency = 30000;            // Minimum time between send (in milliseconds). We don't want to spam the gateway.
volatile unsigned int highThreshold[nEM] = {500,500}; // higher threshold for analog readings
volatile unsigned int lowThreshold[nEM] = {400,400};  // lower threshold for analog readings
volatile uint32_t pulseCount[nEM] = {0};
volatile unsigned long lastBlink[nEM] = {0};
volatile double watt[nEM] = {0};
volatile int high[nEM] = {0};
volatile int low[nEM] = {0};
boolean pcReceived[nEM] = {false};
unsigned long oldPulseCount[nEM] = {0};
unsigned long gwPulseCount[nEM] = {0};
double ppw = ((double)PULSE_FACTOR) / 1000;       // Pulses per watt
double oldwatt[nEM] = {0};
double kWh = 0;
double oldkWh[nEM] = {0};
unsigned long lastSend = 0;
unsigned long lastPulse[nEM] = {0};
int sensorValue[nEM];
boolean sensorState[nEM];
int i = 0;

// command line
#define MAX_CMD_LEN 15
char command[MAX_CMD_LEN]; // command buffer
int inCount = 0; // command buffer index
boolean cmdComplete = false;

// Mode of serial line: 
// C - command, D - data output
char mode = 'D';

// Data output mode:
// R - raw data
// T - trigger events
// C - counter
char dataOutput = 'T';

void setup()
{
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion) 
  //pinMode(DIGITAL_INPUT_SENSOR1, INPUT_PULLUP);
  //pinMode(DIGITAL_INPUT_SENSOR2, INPUT_PULLUP);

  // Fetch last known pulse count value from gw
  request(CHILD_ID1, V_VAR1);
  wait(50);
  request(CHILD_ID2, V_VAR1);
  wait(50);
  
  i = 0;
  lastSend = millis();
  
  while(i <= (nEM-1)) {
    pulseCount[i] = oldPulseCount[i] = 0;
    //lastPulse[i] = millis();
        
    // Fetch the last set thresholds from EEPROM
    high[i] = readEeprom(0+(i*2));
    low[i] = readEeprom(4+(i*2));
    
    if (high[i] > 0 || low[i] > 0) {
      highThreshold[i] = high[i];
      Serial.print("High threshold Energymeter");
      Serial.print(i+1);
      Serial.print(" fetched from EEPROM: ");
      Serial.println(highThreshold[i]);
      lowThreshold[i] = low[i];
      Serial.print("Low threshold Energymeter");
      Serial.print(i+1);
      Serial.print(" fetched from EEPROM: ");
      Serial.println(lowThreshold[i]);
      // Send Settings to gateway
      if (i == 0) {
        send(highthresholdMsg1.set(highThreshold[0], 0));
        wait(50);
        send(lowthresholdMsg1.set(lowThreshold[0], 0));
        wait(50);
      }
      if (i == 1) {
        send(highthresholdMsg2.set(highThreshold[1], 0));
        wait(50);
        send(lowthresholdMsg2.set(lowThreshold[1], 0));
        wait(50);
      }
    } 
    else {
      Serial.print("High threshold Energymeter");
      Serial.print(i+1);
      Serial.print(" set to standard value: ");
      Serial.println(highThreshold[i]);
      Serial.print("Low threshold Energymeter");
      Serial.print(i+1);
      Serial.print(" set to standard value: ");
      Serial.println(lowThreshold[i]);
      // Send standart Settings to gateway
      if (i == 0) {
        send(highthresholdMsg1.set(highThreshold[0], 0));
        wait(50);
        send(lowthresholdMsg1.set(lowThreshold[0], 0));
        wait(50);
      }
      if (i == 1) {
        send(highthresholdMsg2.set(highThreshold[1], 0));
        wait(50);
        send(lowthresholdMsg2.set(lowThreshold[1], 0));
        wait(50);
      }              
    }
  i++;
  }
  i = 0;
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VER);

  // Register this device as Energymeter sensor
  present(CHILD_ID1, S_POWER);
  present(CHILD_ID2, S_POWER);
}

void loop()
{
  if (mode == 'C') {
    doCommand();
  } else if (mode == 'D') {
    if (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == 'C') {
        // exit data mode
        mode = 'C';
      }
    }
  }
  if (mode == 'D') {
    unsigned long currentTime = millis();    
    // Check the analog sensor values and change state when thresholds are passed
    checkThreshold(0);
    checkThreshold(1);

    // Only send values at a maximum frequency or woken up from sleep
    if (SLEEP_MODE || (currentTime - lastSend > sendFrequency)) {
      lastSend = currentTime;
  
        //Last Pulsecount not yet received from controller, request it again
        if (!pcReceived[0] || !pcReceived[1]){
          if (pcReceived[0] = false) {
            request(CHILD_ID1, V_VAR1);
          }
          if (pcReceived[1] = false) {
            request(CHILD_ID2, V_VAR1);
          }          
          return;
        }
        // Send Watt if watt has a new value
        if ((!SLEEP_MODE && watt[0] != oldwatt[0]) || (!SLEEP_MODE && watt[1] != oldwatt[1])) {
          if (watt[0] != oldwatt[0]){
            oldwatt[0] = watt[0];
            Serial.print("Watt_Eenergymeter1:");
            Serial.println(watt[0]);
            // Check that we dont get unreasonable large watt value. 
            // could hapen when long wraps or false interrupt triggered
            if (watt[0] < ((unsigned long)MAX_WATT)) {
             // Send watt Energymeter1 value to gw
              send(wattMsg1.set(watt[0], 2));
            }
          } 
          if (watt[1] != oldwatt[1]){
            oldwatt[1] = watt[1];
            Serial.print("Watt_Eenergymeter2:");
            Serial.println(watt[1]);
            // Check that we dont get unreasonable large watt value. 
            // could hapen when long wraps or false interrupt triggered
            if (watt[1] < ((unsigned long)MAX_WATT)) {
              // Send watt Energymeter2 value to gw
              send(wattMsg2.set(watt[1], 2));
            }
          }
        }
        
        // No Pulse count received in 8 min (Powerconsumtion is lower then 100W (at Energymeter 75 rpm/kWh)
        if (currentTime - lastPulse[0] > 480000) {
          watt[0] = 0;
        }
        if (currentTime - lastPulse[1] > 480000) {
          watt[1] = 0;
        }
        
        // Pulse count has changed
        if (((pulseCount[0] != oldPulseCount[0]) || (!SLEEP_MODE)) || ((pulseCount[1] != oldPulseCount[1]) || (!SLEEP_MODE))) {
          if (pulseCount[0] != oldPulseCount[0]){
            oldPulseCount[0] = pulseCount[0];
            Serial.print("pulsecount Energymeter1: ");
            Serial.println(pulseCount[0]);
            send(lastCounterMsg1.set(pulseCount[0], 0));
            double kWh = (pulseCount[0] / ((double)PULSE_FACTOR));
            if ((kWh != oldkWh[0]) || (!SLEEP_MODE)) {
              oldkWh[0] = kWh;
              Serial.print("Energymeter1 kWh: ");
              Serial.println(kWh, 3);
              send(kwhMsg1.set(kWh, 3));
            }
          }  
          if (pulseCount[1] != oldPulseCount[1]){
            oldPulseCount[1] = pulseCount[1];
            Serial.print("pulsecount Energymeter2: ");
            Serial.println(pulseCount[1]);
            send(lastCounterMsg2.set(pulseCount[1], 0));
            double kWh = (pulseCount[1] / ((double)PULSE_FACTOR));
            if ((kWh != oldkWh[1]) || (!SLEEP_MODE)) {
              oldkWh[1] = kWh;
              Serial.print("Energymeter2 kWh: ");
              Serial.println(kWh, 3);
              send(kwhMsg2.set(kWh, 3));
            }
          }
        }
      }
  }
  if (SLEEP_MODE) {
      sleep(sendFrequency);
  }    
  delay(10);                     
}

void doCommand() {
  // Read one line from serial connection and interpret it as a command
  // print prompt 
  Serial.print(">");
  while (! cmdComplete) {
    // read input
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      if (inChar == '\n' || inChar == '\r') {
        command[inCount] = 0;
        Serial.println();
        cmdComplete = true;
        break; // End of line
      } else if (inCount < MAX_CMD_LEN - 1) {
        command[inCount] = inChar;
        inCount++;
        // echo
        Serial.print(inChar);
      }
    }
  }
  
  // interprete command
  switch (command[0]) {
    case 'D':
        // start raw data mode 
        mode = 'D';
        dataOutput = 'R';
    break;
    case 'T':
      // start trigger data mode
      mode = 'D';
      dataOutput = 'T';
    break;
    case 'S':
    switch (command[1]) {
      case '1':
        // set trigger levels
        setTriggerLevels(0);
        high[0] = readEeprom(0);
        low[0] = readEeprom(4);
      break;
      case '2':
        // set trigger levels
        setTriggerLevels(1);
        high[1] = readEeprom(2);
        low[1] = readEeprom(6);
      break;
    }
    break;
    case 'C':
      Serial.println("Please enter Mode (D = Data Mode (Analog Values) / T = Trigger Mode (Trigger Events) / S1 or S2 = Set Treshold e.G S1 200 300)!");
    break;
  }
  
  // clear command buffer
  command[0] = 0;
  inCount = 0;
  cmdComplete = false;
}

void setTriggerLevels(int nSens) {
  char *p = &command[2];                  
  while (*p != 0 && *p == ' ') {              
    ++p;                          
  }
  char *q = p + 1;
  while (*q != 0 && *q != ' ') {
    ++q;
  }
  *q = 0;
  if (nSens == 0){
    storeEeprom(4, atoi(p));
    Serial.print("Stored new low threshold Energymeter1 to eeprom: ");
    Serial.println(p);
    send(lowthresholdMsg1.set(atoi(p), 0));
    wait(100);
  }
  if (nSens == 1){
    storeEeprom(6, atoi(p));
    Serial.print("Stored new low threshold Energymeter2 to eeprom: ");
    Serial.println(p);
    send(lowthresholdMsg2.set(atoi(p), 0));
    wait(100);
  }
    
  p = q + 1;
  while (*p != 0 && *p == ' ') {
    ++p;
  }
  q = p + 1;
  while (*q != 0 && *q != ' ') {
    ++q;
  }
  *q = 0;
  if (nSens == 0){
    storeEeprom(0, atoi(p));
    Serial.print("Stored new high threshold Energymeter1 to eeprom: ");
    Serial.println(p);
    send(highthresholdMsg1.set(atoi(p), 0));
    wait(100);
  }
  if (nSens == 1){
    storeEeprom(2, atoi(p));
    Serial.print("Stored new high threshold Energymeter2 to eeprom: ");
    Serial.println(p);
    send(highthresholdMsg2.set(atoi(p), 0));
    wait(100);
  }  
}

void storeEeprom(int pos, int value) {
  // function for saving the values to the internal EEPROM
  // value = the value to be stored (as int)
  // pos = the first byte position to store the value in
  // only two bytes can be stored with this function (max 32.767)
  saveState(pos, ((unsigned int)value >> 8));
  pos++;
  saveState(pos, (value & 0xff));
}

int readEeprom(int pos) {
  // function for reading the values from the internal EEPROM
  // pos = the first byte position to read the value from 

  int hiByte;
  int loByte;

  hiByte = loadState(pos) << 8;
  pos++;
  loByte = loadState(pos);
  return (hiByte | loByte);
}

void receive(const MyMessage &message)
{
  if (message.type == V_VAR1) {
    switch (message.sensor) {
      case 1: 
          gwPulseCount[0] = message.getULong();
          pulseCount[0] += gwPulseCount[0];
          watt[0] = oldwatt[0] = 0;
          Serial.print("Received last pulse count Energymeter1 from gw: ");
          Serial.println(pulseCount[0]);
          pcReceived[0] = true;
        break;
        case 2:
          gwPulseCount[1] = message.getULong();
          pulseCount[1] += gwPulseCount[1];
          watt[1] = oldwatt[1] = 0;
          Serial.print("Received last pulse count Energymeter2 from gw: ");
          Serial.println(pulseCount[1]);
          pcReceived[1] = true;
        break;      
        default: 
        Serial.print("Received invalid value from gw!");
     }
  }
}


void checkThreshold(int nSens) 
{
   sensorValue[nSens] = getAverage(nSens);
   
   switch (dataOutput) {
    case 'R':
      Serial.print("val EnergyMeter");
      Serial.print(nSens+1);
      Serial.print("= ");
      Serial.println(sensorValue[nSens], DEC);
    break;
    case 'T':
      // detect and output trigger
      if ((sensorState[nSens] == true) && (sensorValue[nSens] < lowThreshold[nSens])) {
        // Powercalculation
        uint32_t newBlink = {micros()};
        uint32_t interval = {newBlink - lastBlink[nSens]};
        if (interval != 0) {
          lastPulse[nSens] = millis();
          watt[nSens] = (3600000000.0 /interval) / ppw;
        }
        lastBlink[nSens] = newBlink;
        //Increase pulseCount
        pulseCount[nSens]++;
        Serial.print("pulsCount Energymeter");
        Serial.print(nSens+1);
        Serial.println(" + 1");
        sensorState[nSens] = !sensorState[nSens];
        Serial.print("Sensor state Energymeter");
        Serial.print(nSens+1);
        Serial.print(": ");
        Serial.println(sensorState[nSens],DEC);
      }
      if ((sensorState[nSens] == false) && (sensorValue[nSens] > highThreshold[nSens])) {
        sensorState[nSens] = !sensorState[nSens];
        Serial.print("Sensor state Energymeter");
        Serial.print(nSens+1);
        Serial.print(": ");
        Serial.println(sensorState[nSens],DEC);
      }
    break;
  }     
}

int getAverage(int nSens)
{
  int cycles = 0;
  int val = 0;       // variable to store current values from the input
  int newVal = 0;    // variable to store new values from the input
  int average = 0;   // variable to store the peak value
  int count = 0;     // variable for loop

              // take 10 samples over quarter a second 
              // for each sample the input is taken 10 times
  for (cycles = 0; cycles < 10; cycles++) {
    // read input 10 times and get the sum
    // "ANALOG_INPUT_SENSOR" was defined in main program as shown below:
    // "#define ANALOG_INPUT_SENSOR A0"
    for (count = 0; count < 10; count++) {
      switch (nSens) {
            case 0:
              val = val + analogRead(ANALOG_INPUT_SENSOR1);
            break;
            case 1:
              val = val + analogRead(ANALOG_INPUT_SENSOR2);
            break;
          }
      }
    // get average of readings
    val = (val / 10);
    
    // add reading to newVal
    newVal += val;
    val = 0;
    // measure samples over quarter a second or "newVal" will
    // almost always be the same resulting in wrong average
    wait(25); // 10 cycles of 25ms gives  10 samples in 250ms
  }
  // set average to average of newValue1
  average = newVal / 10;
 
  // return the value of average to main program
  return average;
}
