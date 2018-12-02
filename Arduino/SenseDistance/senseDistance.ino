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
// Hardware
#include "hcsr04.h"
// Configuration
#include "def_pin_definitions.h"
#include "def_configuration_constants.h"
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
}

void loop() 
{
	// Measure the distance then report it
	int left_dist_mm = left_distance_sensor.distanceInMillimeters();
	int right_dist_mm = right_distance_sensor.distanceInMillimeters();
	reportData(left_dist_mm, right_dist_mm);

	// Wait until the next scheduled measurement
	while (!checkTime(start_time, MEASUREMENT_PERIOD))
	{
		// Do nothing
	}
	// Update the start_time check
	start_time = start_time + MEASUREMENT_PERIOD * MS_PER_S;
}