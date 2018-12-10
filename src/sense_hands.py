#!/usr/bin/env python
#######################################
# Import Required Libraries
import rospy
import numpy as np
import serial
import serial.tools.list_ports

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
	# remove end-line characters
	data_in = data_in[:-2]
	splitData = data_in.split(',')
	return [int(splitData[0]), int(splitData[1])]
	#[int(s) for s in data_in.split(',')]



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
	# find Serial ports
	sPorts = [comport.device for comport in serial.tools.list_ports.comports()]
	print(sPorts)

	# Open the com port
	ser = serial.Serial()
	ser.baudrate = 115200
	ser.port = sPorts[0]

	try:
		ser.close()
		ser.open()
		ser.reset_input_buffer()
	except:
		print("Error accessing serial port")
		exit()

	# Perform time setup
	# None needed. Tick rate is defined on the arduino

	while not rospy.is_shutdown():
		# Wait for message on Serial
		try:
			data_in = ser.readline()
		except:
			pass

		# interpret the data
		try:
			[meas_left, meas_right] = process_incoming_data(data_in)

			meas_left = adjust_measurement(meas_left,calib_distsensor_left)
			meas_right = adjust_measurement(meas_right,calib_distsensor_right)

			##print([meas_left,meas_right])
			# Publish the measurements
			hand_positions.left_distance = meas_left
			hand_positions.right_distance = meas_right

			hand_position_publisher.publish(hand_positions)
		except:
			pass



#########################################
# Boilerplate Code
if __name__ == '__main__':
	try:
		publish_measurements()
	except rospy.ROSInterruptException:
		pass
0
