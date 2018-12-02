// Author: Petras Swissler
// Created: 2018-12-01
/////////////////////////////////////////////////////////////////////////
// This file creates the global variables used in the setup() and loop()
// Dependencies: hcsr04.h
/////////////////////////////////////////////////////////////////////////
// Create Left and Right Distance Sensors
HCSR04 left_distance_sensor(PIN_LEFT_TRIG, PIN_LEFT_ECHO, RANGE_MIN, RANGE_MAX);
HCSR04 right_distance_sensor(PIN_RIGHT_TRIG, PIN_RIGHT_ECHO, RANGE_MIN, RANGE_MAX);

// Timing variables
unsigned int start_time = 0;