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
* Version 0.2 - Armin Kaiser
*
* DESCRIPTION
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
#define SKETCH_VER            "0.2"				    // Sketch version
#define SKETCH_NAME           "Energy Meter"	// Optional child sensor name
#define CHILD_ID              1					      // Id of the sensor child
#define CHILD_NAME            "Energy Meter"	// Optional child sensor name
#define NODE_REPEATER         false				    // Set true if a repeater node (i.e. always turned on)
#define SLEEP_MODE            false				    // powervalue can only be reported when sleep mode is false.
//#define MY_NODE_ID          101             // Node ID if RS485 (Change NODE_ID to your needs)

// Sketch settings
#define PULSE_FACTOR          75				      // Number of blinks/rpm per kWh of your meter (75 rotation/kWh)
#define MAX_WATT 			        11000				    // Max watt value to report. This filters outliers.	(heat pump  ~ 10kW) (11KWh ~825 U/h ~ 0,23 U/s)

// Input and output definitions
#define DIGITAL_INPUT_SENSOR  3					      // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define ANALOG_INPUT_SENSOR   A0				      // The analog input you you attached your sensor. 

MyMessage wattMsg(CHILD_ID,V_WATT);
MyMessage kwhMsg(CHILD_ID,V_KWH);
MyMessage lastCounterMsg(CHILD_ID, V_VAR1);
MyMessage highthresholdMsg(CHILD_ID,V_VAR3);
MyMessage lowthresholdMsg(CHILD_ID,V_VAR4);

// Global vars
unsigned long sendFrequency = 30000;			    // Minimum time between send (in milliseconds). We don't want to spam the gateway.
volatile unsigned int highThreshold = 500;		// higher threshold for analog readings
volatile unsigned int lowThreshold = 400;     // lower threshold for analog readings
volatile uint32_t pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double watt = 0;
volatile int high = 0;
volatile int low= 0;
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double ppw = ((double)PULSE_FACTOR) / 1000;		// Pulses per watt
double oldwatt = 0;
double kWh = 0;
double oldkWh = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;
int sensorValue;
boolean sensorState;

// command line
#define MAX_CMD_LEN 80
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
	pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);

	pulseCount = oldPulseCount = 0;

	// Fetch last known pulse count value from gw
	request(CHILD_ID, V_VAR1);

	lastSend = lastPulse = millis();

	// Fetch the last set thresholds from EEPROM
	high = readEeprom(0); //loadState (0);
	low = readEeprom(2); //loadState (2);
	if (high > 0 || low > 0) {
		highThreshold = high;
		lowThreshold = low;
		debugMessage("High threshold fetched from EEPROM, value: ", String(highThreshold));
		debugMessage("Low threshold fetched from EEPROM, value: ", String(lowThreshold));
	}
	else {
		debugMessage("High threshold set to standard value: ", String(highThreshold));
		debugMessage("Low threshold set to standard value: ", String(lowThreshold));
		
	}
 // Send Settings to gateway
 wait(50);
 send(highthresholdMsg.set(highThreshold, 0));
 wait(50);
 send(lowthresholdMsg.set(lowThreshold, 0));
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_VER);

	// Register this device as Energymeter sensor
	present(CHILD_ID, S_POWER);
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
		checkThreshold();
    
		// Only send values at a maximum frequency or woken up from sleep
		if (SLEEP_MODE || (currentTime - lastSend > sendFrequency)) {
			lastSend = currentTime;

			if (!pcReceived) {
				//Last Pulsecount not yet received from controller, request it again
				request(CHILD_ID, V_VAR1);
				return;
			}

			if (!SLEEP_MODE && watt != oldwatt) {
				oldwatt = watt;
				debugMessage("Watt:", String(watt));

				// Check that we dont get unreasonable large watt value. 
				// could hapen when long wraps or false interrupt triggered
					if (watt < ((unsigned long)MAX_WATT)) {
						// Send watt value to gw
						send(wattMsg.set(watt, 2));
					}
			}

			// No Pulse count received in 8 min (Powerconsumtion is lower then 100W (at Energymeter 75 rpm/kWh)
			if (currentTime - lastPulse > 480000) {
				watt = 0;
			}

			// Pulse count has changed
			if ((pulseCount != oldPulseCount) || (!SLEEP_MODE)) {
				oldPulseCount = pulseCount;
				debugMessage("pulsecount: ", String(pulseCount));
			}
			double kWh = ((double)pulseCount / ((double)PULSE_FACTOR));
			if ((kWh != oldkWh) || (!SLEEP_MODE)) {
				oldkWh = kWh;
				debugMessage("kWh: ", String(kWh, 3));
			
			}
		}
	}
	if (SLEEP_MODE) {
		  sleep(sendFrequency);
	}    
  
  delay(10);                     
}

void debugMessage(String header, String content)
{
	// DEBUG code ------
	Serial.print(header);
	Serial.println(content);
	// DEBUG code ------   
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
      // set trigger levels
      setTriggerLevels();
      high = readEeprom(0); //loadState (0);
	    low = readEeprom(2); //loadState (2);
      break;
    case 'C':
      Serial.println("Please enter Mode (D = Data Mode (Analog Values) / T = Trigger Mode (Trigger Events) / S = Set Treshold e.G S 200 300)!");
      break;
  }
  
  // clear command buffer
  command[0] = 0;
  inCount = 0;
  cmdComplete = false;
}

void setTriggerLevels() {
	// Set trigger levels (low high) from the command line and store them into EEPROM
	char *p = &command[1];
	while (*p != 0 && *p == ' ') {
		++p;
	}
	char *q = p + 1;
	while (*q != 0 && *q != ' ') {
		++q;
	}
	*q = 0;
	storeEeprom(2, atoi(p));
	debugMessage("Stored new low threshold to eeprom: ", p);
	send(lowthresholdMsg.set(atoi(p), 0));
  
	p = q + 1;
	while (*p != 0 && *p == ' ') {
		++p;
	}
	q = p + 1;
	while (*q != 0 && *q != ' ') {
		++q;
	}
	*q = 0;
	storeEeprom(0, atoi(p));
	debugMessage("Stored new high threshold to eeprom: ", p);
	send(highthresholdMsg.set(atoi(p), 0));
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
	switch (message.type) {
		case V_VAR1: {
			unsigned long gwPulseCount = message.getULong();
			pulseCount += gwPulseCount;
			watt = oldwatt = 0;
			debugMessage("Received last pulse count from gw: ", String(pulseCount));
			pcReceived = true;
	    }
		break;
		default: {
		  debugMessage("Received invalid message from gw! ", "");
		}
	}
}

void checkThreshold() 
{
	sensorValue = getAverage();
	switch (dataOutput) {
		case 'R':
			// print the raw data to the serial monitor         
			debugMessage("val = ", String(sensorValue, DEC));
		break;
		case 'T':
			// detect and output trigger
			if ((sensorState == true) && (sensorValue < lowThreshold)) {
				// Powercalculation
				uint32_t newBlink = micros();
				uint32_t interval = newBlink-lastBlink;
				if (interval != 0) {
					lastPulse = millis();
					watt = (3600000000.0 /interval) / ppw;
				}
				lastBlink = newBlink;
				//Increase pulseCount
				pulseCount++;
				debugMessage("pulsCount + 1", "");
		    	sensorState = !sensorState;
				debugMessage("Sensor state: ", String(sensorState));
			}
		    if ((sensorState == false) && (sensorValue > highThreshold)) {
				sensorState = !sensorState;
				debugMessage("Sensor state: ", String(sensorState));
		    }
		break;
	}
}

int getAverage()
{
	int cycles = 0;
	int val = 0;		// variable to store current values from the input
	int newVal = 0;     // variable to store new values from the input
	int average = 0;    // variable to store the peak value
	int count = 0;      // variable for loop

						// take 10 samples over quarter a second 
						// for each sample the input is taken 10 times
	for (cycles = 0; cycles < 10; cycles++) {
		// read input 10 times and get the sum
		// "ANALOG_INPUT_SENSOR" was defined in main program as shown below:
		// "#define ANALOG_INPUT_SENSOR A0"
		for (count = 0; count < 10; count++) {
			val = val + analogRead(ANALOG_INPUT_SENSOR);
		}
		// get average of readings
		val = (val / 10);

		// add reading to newVal
		newVal += val;
		val = 0;
		// measure samples over quarter a second or "newVal" will
		// almost always be the same resulting in wrong average
		delay(25); // 10 cycles of 25ms gives  10 samples in 250ms
	}
	// set average to average of newValue1
	average = newVal / 10;

	// return the value of average to main program
	return average;
}
