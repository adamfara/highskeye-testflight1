/*
* highskeye_testflight1.ino
*
* Created: 11/9/2014 1:17:09 AM
* Author: Adam
*/

#include <Wire.h>
#include <DynamixelSerial.h>
#include <BMP085.h>
#include <SendOnlySoftwareSerial.h>
#include "presclaler.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define set(reg,bit)		reg |= (1<<(bit))
#define clear(reg,bit)		reg &= ~(1<<(bit))
#define toggle(reg,bit)		reg ^= (1<<(bit))

//
// Timing
//
float dt = 0.;
float flight_time = 0.;
uint32_t cycle_curr;
uint32_t cycle_prev;

//
// Dynamixel stuff
//
#define DYN_VALVE_OPEN		550
#define DYN_VALVE_CLOSE		775
#define DYN_COM_DIR_PIN		2
#define DYN_BAUD_RATE		115200
uint16_t dynPosition;
uint8_t dynPosChangeFlag;
uint8_t dynLEDFlag;

//
// State Machine
//
uint8_t state;
#define S_FILL				10
#define S_ASCENT			11
#define S_CRUISE			12
#define S_DESCENT			13

//
// Manual interface
//
#define BUTTON_1_PIN		3
#define BUTTON_2_PIN		4
#define BUTTON_3_PIN		5
uint8_t b1_state;
uint8_t b2_state;
uint8_t b3_state;

#define LED_ONBOARD_PIN		13
#define LED_GREEN_PIN		9
#define LED_RED_PIN			8

//
// Pressure Sensor and related data handling
//
#define ALT_AVG_FILTER_DEPTH	10
float altitudeAvgFilter[ALT_AVG_FILTER_DEPTH];
float altitudeAvgFilterTotal;
uint8_t altitudeAvgFilterIndex;

#define ALT_LOW_PASS_BETA		0.85
float altitudeLowPassed;

int8_t temperature;
int32_t pressure;
float altitude;

//
// SoftwareSerial and OpenLog
//
#define OPENLOG_SOFT_TX			7
#define OPENLOG_SOFT_BAUD		115200
SendOnlySoftwareSerial logSerial(OPENLOG_SOFT_TX);

//
// Flight Control Parameters
//
// For test 1, we rise to 100 ft and open the valve, hoping for a steady descent to follow
// 100 ft = 30.48m
#define VALVE_OPEN_THRESH		30.48


void setup()
{
	// Timing
	//Timer 1 Control Timer Setup
	clear(TCCR1B, CS12);	//set timer to system clock/64 (250 kHz)
	set(TCCR1B, CS11);
	set(TCCR1B, CS10);
	
	clear(TCCR1B, WGM13);	//counts to 0xFFFF then reset
	clear(TCCR1B, WGM12);
	clear(TCCR1A, WGM11);
	clear(TCCR1A, WGM10);
	
	clear(TCCR1A, COM1A1);	//no change on the C6 pin
	clear(TCCR1A, COM1A0);
	
	cycle_curr = TCNT1;
	cycle_prev = TCNT1;
	
	// Dynamixel
	dynCloseValve();
	
	// Manual Interface
	pinMode(BUTTON_1_PIN, INPUT);
	pinMode(BUTTON_2_PIN, INPUT);
	pinMode(BUTTON_3_PIN, INPUT);
	
	pinMode(LED_ONBOARD_PIN, OUTPUT);
	digitalWrite(LED_ONBOARD_PIN, LOW);
	pinMode(LED_GREEN_PIN, OUTPUT);
	digitalWrite(LED_GREEN_PIN, LOW);
	pinMode(LED_RED_PIN, OUTPUT);
	digitalWrite(LED_RED_PIN, LOW);
	
	// Pressure Sensor
	BMP085.begin(0);
	temperature = BMP085.getTemperature(BMP085.readUT());
	pressure = BMP085.getPressure(BMP085.readUP());
	altitude = BMP085.getAltitude(pressure);
	
	altitudeLowPassed = altitude;
	for (int i = 0; i < ALT_AVG_FILTER_DEPTH; i++) {
		altitudeAvgFilter[i] = altitude;
	}
	altitudeAvgFilterTotal = altitude * ALT_AVG_FILTER_DEPTH;
	altitudeAvgFilterIndex = 0;
	
	// State Machine
	state = S_FILL;
	
	// Logging
	logSerial.begin(OPENLOG_SOFT_BAUD);
	for (int i = 0; i < 10; i++) {
		logSerial.println("University of Pennsylvania - Test Flight 1 - Contact Adam Farabaugh 412.913.7031 if found.  Reward offered!");
	}
}

void loop() {
	// Timing
	cycle_curr = TCNT1;
	dt = (float)(cycle_curr - cycle_prev) / 250000.0;
	flight_time += dt;
	cycle_prev = cycle_curr;
	
	// Dynamixel
	if (dynPosChangeFlag) {
		dynPosChangeFlag = 0;
		Dynamixel.begin(DYN_BAUD_RATE, DYN_COM_DIR_PIN);
		Dynamixel.move(1, dynPosition);
	}
	
	// Manual Interface
	b1_state = digitalRead(BUTTON_1_PIN);
	b2_state = digitalRead(BUTTON_2_PIN);
	b3_state = digitalRead(BUTTON_3_PIN);
	
	// Pressure Sensor
	temperature = BMP085.getTemperature(BMP085.readUT());
	pressure = BMP085.getPressure(BMP085.readUP());
	altitude = BMP085.getAltitude(pressure);
	
	altitudeAvgFilterTotal -= altitudeAvgFilter[altitudeAvgFilterIndex];
	altitudeAvgFilter[altitudeAvgFilterIndex++] = altitude;
	altitudeAvgFilterTotal += altitude;	
	altitudeAvgFilterIndex = altitudeAvgFilterIndex % ALT_AVG_FILTER_DEPTH;
	
	altitudeLowPassed = ALT_LOW_PASS_BETA * altitudeLowPassed + (1 - ALT_LOW_PASS_BETA) * altitude;
	
	// State Machine
	switch(state) {
		case S_FILL:
			if (b1_state && b2_state) {
				digitalWrite(LED_GREEN_PIN, HIGH);
				digitalWrite(LED_RED_PIN, HIGH);
				dynCloseValve();
				state = S_ASCENT;
			}
			else if (b2_state) {
				dynCloseValve();
			}
			else if (b1_state) {
				dynOpenValve();
			}
			break;
		case S_ASCENT:
			if ((altitudeAvgFilterTotal / ALT_AVG_FILTER_DEPTH) > VALVE_OPEN_THRESH || (altitudeLowPassed > VALVE_OPEN_THRESH)) {
				dynOpenValve();
				state = S_CRUISE;
			}
			break;
		case S_CRUISE:
			dynOpenValve();
			state = S_DESCENT;
			break;
		case S_DESCENT:
			dynOpenValve();
			break;
		default:
			break;
	}
	
	// Logging
	
}


void dynOpenValve() {
	if (dynPosition != DYN_VALVE_OPEN) {
		dynPosition = DYN_VALVE_OPEN;
		dynPosChangeFlag = 1;
		dynLEDFlag = 1;
	}
}


void dynCloseValve() {
	if (dynPosition != DYN_VALVE_CLOSE) {
		dynPosition = DYN_VALVE_CLOSE;
		dynPosChangeFlag = 1;
		dynLEDFlag = 0;
	}
}

void debugToLog() {
	debugToLogHelper();
	logSerial.println("");
}

void debugToLog(String message) {
	debugToLogHelper();
	logSerial.println(message);
}

void debugToLogHelper() {
	logSerial.print(flight_time, 2);
	logSerial.print(",");
	logSerial.print(dt, 3);
	logSerial.print(",");
	logSerial.print(altitude, 3);
	logSerial.print(",");
	logSerial.print(altitudeAvgFilterTotal / ALT_AVG_FILTER_DEPTH, 2);
	logSerial.print(",");
	logSerial.print(altitudeLowPassed, 2);
	logSerial.print(",");
	logSerial.print(pressure);
	logSerial.print(",");
	logSerial.print(temperature);
	logSerial.print(",");
	logSerial.print(dynPosition);
	logSerial.print(",");
	logSerial.print(dynPosChangeFlag);
	logSerial.print(",");
	logSerial.print(state);
	logSerial.print(",");
	logSerial.print(b1_state);
	logSerial.print(",");
	logSerial.print(b2_state);
	logSerial.print(",");
	logSerial.print(b3_state);
}
