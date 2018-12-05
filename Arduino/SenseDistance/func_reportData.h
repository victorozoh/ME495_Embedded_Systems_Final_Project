// Author: Petras Swissler
// Created: 2018-12-01
/////////////////////////////////////////////////////////////////////////
// This file reports the data in an expected format
// Dependencies: arduino.h
/////////////////////////////////////////////////////////////////////////
void reportData(int left_dist_mm, int right_dist_mm)
{
	Serial.print(left_dist_mm);
	Serial.print(",");
	Serial.print(right_dist_mm);
	Serial.print("\r\n");
}