// Author: Petras Swissler
// Created: 2018-12-01

/////////////////////////////////////////////////////////////////////////
// This file measures the positions of players' hands
// Dependencies:	hcsr04.h, pin_definitions.h
// Hardware:		Two HC-SR04 Sensors
// Inputs:			2x Echo Pins (from sensor).
// Outputs:			Serial out. Formatting: "leftHandPosition(mm), RightHandPosition(mm)"
//								Baud Rate:	115200

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
	Serial.begin(BAUD_RATE);
	start_time = millis();

	pinMode(PIN_LED_LEFT, OUTPUT); digitalWrite(PIN_LED_LEFT, LOW);
	pinMode(PIN_LED_RIGHT, OUTPUT); digitalWrite(PIN_LED_RIGHT, LOW);
	
}

// Global Variables because why not?
int left_dist_mm=50;
int right_dist_mm=50;
float alpha = 0.5;

void loop() 
{
	int meas;
	// Measure the distance then report it
	meas = left_distance_sensor.distanceInMillimeters();
	if (meas > 0)
	{
		left_dist_mm = (float)meas * alpha + (float)left_dist_mm * (1-alpha);
		digitalWrite(PIN_LED_LEFT, LOW);
	}
	else
	{
		// Nothing; keep previous left dist
		digitalWrite(PIN_LED_LEFT, HIGH);
	}
	
	meas = right_distance_sensor.distanceInMillimeters();
	if (meas > 0)
	{
		right_dist_mm = (float)meas * alpha + (float)right_dist_mm * (1-alpha);
		digitalWrite(PIN_LED_RIGHT, LOW);
	}
	else
	{
		// Nothing; keep previous left dist
		digitalWrite(PIN_LED_RIGHT, HIGH);
	}

	reportData(left_dist_mm, right_dist_mm);
	/*
	// Wait until the next scheduled measurement
	while (!checkTime(start_time, MEASUREMENT_PERIOD))
	{
		// Do nothing
	}
	// Update the start_time check
	start_time = start_time + MEASUREMENT_PERIOD * MS_PER_S;*/
}
