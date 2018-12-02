// Author: Petras Swissler
// Created: 2018-09-01

/////////////////////////////////////////////////////////////////////////
// This function checks to see if a certain number of seconds has passed
// Dependencies:	arduino.h
// Hardware:		None
// Inputs:			Time that cycle started, desired duration of cycle
// Outputs:			True = Time has elapsed
//					False = Time remains

/////////////////////////////////////////////////////////////////////////
bool checkTime(unsigned long startTime, float targetDuration_seconds)
{
  if((millis()-startTime)>=(targetDuration_seconds*MS_PER_S))
  { 
	  return true; 
  }
  else{
	  return false; 
  }
}