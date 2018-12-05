#!/usr/bin/env python
#######################################
# Import Required Libraries
import rospy
import numpy as np
import serial

# Messages
from sawyer_pong.msg import measured_distances

#######################################
# HARD-CODED VALUES
# none

#######################################
# Helper Functions
def adjust_measurement(measurement, calibration_value):
	return measurement*calibration_value

def process_incoming_data(data_in):
	# Do nothing for now
	return [0,0]


#########################################
# Primary function
def publish_measurements():
	#Initialize the _node
	rospy.init_node('publish_measurements')

	# Get parameters
	calib_distsensor_left = rospy.get_param('~calib_distsensor_left',1)
	calib_distsensor_right = rospy.get_param('~calib_distsensor_right',1)

	arduino_serial_port = rospy.get_param('~measurement_serial_port','COM3')
	arduino_serial_baud = rospy.get_param('~measurement_serial_baud',115200)

	#Create Publishers
	hand_position_publisher = rospy.Publisher('hand_positions', measured_distances, queue_size=1)

	# Create the message variables
	hand_positions = measured_distances()

	# Setup the serial port
	ser = serial.Serial()
	ser.port		= arduino_serial_port
	ser.baudrate	= arduino_serial_baud
	ser.bytesize	= serial.EIGHTBITS
	ser.parity		= serial.PARITY_NONE
	ser.stopbits	= serial.STOPBITS_ONE
	ser.timeout		= 1
	ser.xonxoff		= False
	ser.rtscts		= False
	ser.dsrdtr		= False
	ser.writeTimeout= 2

	ser.reset_input_buffer()

	try:
		ser.close()
		ser.open()
	except:
		print("Error accessing serial port")
		exit()

	# Perform time setup
	# None needed. Tick rate is defined on the arduino

	while not rospy.is_shutdown():		
		# Wait for message on Serial
		data_in = read_until('\n')
		print(data_in)
		
		# interpret the data
		[meas_left, meas_right] = process_incoming_data(data_in)
		meas_left = adjust_measurement(meas_left,calib_distsensor_left)
		meas_right = adjust_measurement(meas_right,calib_distsensor_right)
		
		# Publish the measurements
		hand_positions.left_distance = meas_left
		hand_positions.right_distance = meas_right

		hand_position_publisher.publish(hand_positions)
		


#########################################
# Boilerplate Code
if __name__ == '__main__':
	try:
		publish_measurementsr()
	except rospy.ROSInterruptException:
		pass
0
