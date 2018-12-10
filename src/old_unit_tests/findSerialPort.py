#!/usr/bin/env python
import sys
import glob
import serial
import serial.tools.list_ports

def runMe():
	# find Serial ports
	sPorts = [comport.device for comport in serial.tools.list_ports.comports()]
	print(sPorts)

	# Open the com port
	ser = serial.Serial()
	ser.baudrate = 115200
	ser.port = sPorts[0]
	ser.open()
	while 1:
		print(ser.readline())

if __name__ == '__main__':
    runMe()
