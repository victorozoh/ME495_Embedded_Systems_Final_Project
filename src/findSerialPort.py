#!/usr/bin/env python
import sys
import glob
import serial
import serial.tools.list_ports

def runMe():
<<<<<<< HEAD
	# find Serial ports
	sPorts = [comport.device for comport in serial.tools.list_ports.comports()]
	print(sPorts)
	
	# Open the com port
	ser = serial.Serial()
	ser.baudrate = 115200
	ser.port = sPorts[0]
	ser.open()
	while 1:
		print ser.read(100)
=======
    ser = serial.Serial(serial_ports()[0],115200)
    ser.close()
    ser.open()
    while 1:
        print(ser.readline())
>>>>>>> 4b60c140564df152607861e39a2a8b5b2655cf43


if __name__ == '__main__':
    runMe()
