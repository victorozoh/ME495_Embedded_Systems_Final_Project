// Author: Petras Swissler
// Created: 2018-12-01

/////////////////////////////////////////////////////////////////////////
// This program measures the positions of players' hands and outputs them 
// to serial. An LED will light up if signal is lost.
//
// Dependencies:	hcsr04.h, pin_definitions.h
// Hardware:		Two HC-SR04 Sensors
// Inputs:			2x Echo Pins (from sensor).
// Outputs:			Serial out. Formatting: "leftHandPosition(mm), RightHandPosition(mm)"
//								Baud Rate:	115200
//
/////////////////////////////////////////////////////////////////////////
// Include Libraries
#include <Arduino.h>
// Hardware
#include "hcsr04.h"
// Configuration
#include "def_config_constants.h"
#include "def_pin_definitions.h"
#include "def_global_variables.h"

// Helper Functions
#include "func_checkTime.h"
#include "func_reportData.h"

/////////////////////////////////////////////////////////////////////////
// Include Libraries
void setup() 
{
	// Initialize Serial
	Serial.begin(BAUD_RATE);

	// Time Setup
	start_time = millis();

	// Set up the pins and initialize them low
	pinMode(PIN_LED_LEFT, OUTPUT); digitalWrite(PIN_LED_LEFT, LOW);
	pinMode(PIN_LED_RIGHT, OUTPUT); digitalWrite(PIN_LED_RIGHT, LOW);
}

// Global Variables because why not? (Yes, Jarvis, I know why not.)
int left_dist_mm=50;
int right_dist_mm=50;
float alpha = 0.5; // Controls the weighting of new/ old messages

void loop() 
{
	// Iteratively measure hand distances and print them out over serial

	int meas;
	// Measure the left hand distance 
	meas = left_distance_sensor.distanceInMillimeters();
	if (meas > 0)
	{
		// Measurement smoothing
		left_dist_mm = (float)meas * alpha + (float)left_dist_mm * (1-alpha);
		// Write LED low to signify that measurement obtained
		digitalWrite(PIN_LED_LEFT, LOW);
	}
	else
	{
		// Keep previous left dist. Indicate that no signal seen
		digitalWrite(PIN_LED_LEFT, HIGH);
	}

	// Measure right hand distance
	meas = right_distance_sensor.distanceInMillimeters();
	if (meas > 0)
	{
		// Measurement Smoothing
		right_dist_mm = (float)meas * alpha + (float)right_dist_mm * (1-alpha);
		// Write LED low to signify that measurement obtained
		digitalWrite(PIN_LED_RIGHT, LOW);
	}
	else
	{
		// Keep previous left dist. Indicate that no signal seen
		digitalWrite(PIN_LED_RIGHT, HIGH);
	}

	// Print the data over serial
	reportData(left_dist_mm, right_dist_mm);
}
