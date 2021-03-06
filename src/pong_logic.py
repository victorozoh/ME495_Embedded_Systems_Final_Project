#!/usr/bin/env python
#######################################
# Author: Petras Swissler
# Created: Dec 11, 2018
#######################################
# Import Required Libraries
import 	rospy
import 	numpy as np
#import 	pyfiglet
from 	geometry_msgs.msg import Twist

from pong_plot import *

# Messages
from sawyer_pong.msg import measured_distances

#######################################
# Create Classes
class bound_lrud:
	def __init__(self, xlow, xhigh, yhigh, ylow):
		self.xlow = xlow
		self.xhigh = xhigh
		self.yhigh = yhigh
		self.ylow = ylow

class xy_vector:
	def __init__(self, x, y):
		self.x = x
		self.y = y

#######################################
# Global Variables
hand_positions = [50,50]
ball_positions = xy_vector(50,50)
expected_position=xy_vector(0,0)
combined=xy_vector(0,0)
vel_new=xy_vector(0,0)


y_high = 0.6
y_low = 0.1
x_high = 0.5
x_low = -0.5

plot_size = xy_vector(50,25)

#######################################
# Helper Functions
'''
def disp_score(score):
	pyfiglet.figlet_format('SCORE', font = "drpepper")
	pyfiglet.figlet_format(sprint('%d to %d',score[0],score[1]), font = "drpepper")
	return 1
def disp_win(score):
	if score[0] > score[1]:
		pyfiglet.figlet('LEFT PLAYER WINS')
	else:
		pyfiglet.figlet_format('RIGHT PLAYER WINS')
'''

####
def rand_sign():
	choose = np.random.rand()
	if choose >= 0.5:
		return 1
	else:
		return -1
##
def get_ball_start_velocity(ball_speed):
	xx = (np.random.rand()*0.8 + 0.2)*rand_sign()
	yy = (np.random.rand()*0.4 + 0.0)*rand_sign()

	norm = np.sqrt(xx*xx + yy*yy)
	xx = ball_speed * xx / norm
	yy = ball_speed * yy / norm

	start_vel = xy_vector(xx,yy)
	return start_vel
####
def ball_expected_position(ball_logic_position, ball_cmd_vel, delta_time):


	expected_position.x = ball_logic_position.x + ball_cmd_vel.x * delta_time
	expected_position.y = ball_logic_position.y + ball_cmd_vel.y * delta_time
	return expected_position
####
def combine_ball_positions(simulated, actual):
	simweight = 0.8
	combined.x = simulated.x*simweight + actual.x*(1-simweight)
	combined.y = simulated.y*simweight + actual.y*(1-simweight)
	return combined
####
def check_bounds(position, bound, last_bounce):
	# Init for later logic
	new_bounce = last_bounceprint("hi")

	# Check for collisions
	if position.x > bound.xhigh:
		new_bounce = 'rt'
	if position.x < bound.xlow:
		new_bounce = 'lf'
	if position.y > bound.yhigh:
		new_bounce = 'up'
	if position.y < bound.ylow:
		new_bounce = 'dn'

	if new_bounce != last_bounce:
		do_bounce = True
	else:
		do_bounce = False

	return [do_bounce, new_bounce]
####
def bounce_top(ball_velocity): # THIS takes in xy_vector(xx,yy)
	vel_new.x = ball_velocity.x
	vel_new.y = -1* np.abs(ball_velocity.y)
	#result = pyfiglet.figlet_format("BONK", font = "drpepper")
	return vel_new
def bounce_bottom(ball_velocity): # THIS takes in xy_vector(xx,yy)
	vel_new.x = ball_velocity.x
	vel_new.y = np.abs(ball_velocity.y)
	#result = pyfiglet.figlet_format("BLIP", font = "drpepper")
	return vel_new
def bounce_side_left(ball_velocity, left_hand_position, ball_position, paddle_size):
	if np.abssolute(ball_position.y-left_hand_position) < (paddle_size/2):
		vel_new.y = ball_velocity.y
		vel_new.x = np.abs(ball_velocity.x)
		#result = pyfiglet.figlet_format("BEEP", font = "drpepper")
		return [vel_new, False]
	else:
		vel_new.x = 0
		vel_new.y = 0
		#result = pyfiglet.figlet_format("POINT FOR RIGHT", font = "drpepper")
		return [vel_new, True]
def bounce_side_right(ball_velocity, left_hand_position, ball_position, paddle_size):
	if np.abssolute(ball_position.y-right_hand_position) < (paddle_size/2):
		vel_new.y = ball_velocity.y
		vel_new.x = np.abs(ball_velocity.x)*-1
		#result = pyfiglet.figlet_format("BOOP", font = "drpepper")
		return [vel_new, False]
	else:
		vel_new.x = 0
		vel_new.y = 0
		#result = pyfiglet.figlet_format("POINT FOR LEFT", font = "drpepper")
		return [vel_new, True]

def bounce_the_ball(ball_velocity,last_bounce, left_hand_position, right_hand_position, ball_position, paddle_size):
	score_happened = False
	# Do the bounces
	if last_bounce == 'lf':
		[vel_new, score_happened] = bounce_side_left(ball_velocity, left_hand_position, ball_position, paddle_size)
	if last_bounce == 'rt':
		[vel_new, score_happened] = bounce_side_right(ball_velocity, right_hand_position, ball_position, paddle_size)
	if last_bounce == 'up':
		vel_new = bounce_top(ball_velocity)
	if last_bounce == 'dn':
		vel_new = bounce_bottom(ball_velocity)
		ball_velocity,
	return [vel_new, score_happened]

#######################################
# Callbacks
def hand_position_callback(latest_msg):
	hand_positions[0] = latest_msg.left_distance
	hand_positions[1] = latest_msg.right_distance




#########################################
# Primary functionexit
def pong_logic():
	#Initialize the _node
	rospy.init_node('pong_logic')

	#######################ball_velocity *
	# Get parameters
	# TODO english_amount     = rospy.get_param('~english_amount',0)
	paddle_size        = rospy.get_param('~paddle_size',.2)
	# TODO ball_velocity_incr = rospy.get_param('~ball_velocity_incr',1)
	ball_speed      = rospy.get_param('~ball_velocity',.05)
	max_score          = rospy.get_param('~max_score',1)

	circle_tick_rate = rospy.get_param('~circle_tick_rate', 10)

	#######################
	# Create Publishers
	hand_velocity_publisher = rospy.Publisher('hand_vel', Twist, queue_size=1)

	#TODO: hand_position_publisher = rospy.Publisexit her('hand_vel', hand_vel, queue_size=1)

	#######################
	# Create Subscribers
	# TODO: position_subscriber = rospy.Subscriber('position_subscriber', hand_position, self.scan_callback)
	# TODO: boundary_subscriber = rospy.Subscriber('position_subscriber', hand_position, self.scan_callback)
	hand_position_subscriber = rospy.Subscriber('hand_positions', measured_distances, hand_position_callback)

	#######################
	# Create the message variables
	# TODO: Message_twist
	# TODO: Message_position

	#vel_new_twist=Twist()bounds, plot_size, left_paddle_position, right_paddle_position, paddle_size, ball_position

	#######################
	# Create start variables
	score           = [0,0]
	ball_cmd_vel    = get_ball_start_velocity(ball_speed)
	left_hand_position  = hand_positions[0]
	right_hand_position = hand_positions[1]

	last_bounce = 0

	#######################
	# Perform time setup
	r = rospy.Rate(circle_tick_rate)
	start_time = rospy.get_time()
	now_time = start_time

	# TEMP. DELETE ME
	bounds = bound_lrud(x_low,x_high,y_high,y_low)          # get from Subscriber (or param or paramserver or sub, based on ambition)
	ball_logic_position = xy_vector(0,(bounds.yhigh+bounds.ylow)/2.0)           #get from sub
	#TODO: Put hand at default Position


	#######################
	# Main Loop
	while not rospy.is_shutdown():
		#####################
		# TODO: LIES LIES LIES LIES
		ball_recieved_position = ball_logic_position
		#####################

		# Time Calcs
		last_time   = now_time
		now_time    = rospy.get_time() - start_time
		delta_time  = now_time - last_time

		#		print(left_hand_position) Get Position of robot arm
		# TODO: position_subscriber.get
		# TODO: robo_position = transform_position()


		# Calculate position to use for logic
		ball_logic_position = combine_ball_positions(ball_expected_position(ball_logic_position, ball_cmd_vel, delta_time), ball_recieved_position)

		# Get Hand Positions
		left_hand_position  = hand_positions[0]/1000.0
		right_hand_position = hand_positions[1]/1000.0

		print(right_hand_position, left_hand_position)

		"""
		# Check Bounds
		[do_bounce,last_bounce] = check_bounds(ball_logic_position, bounds, last_bounce)
		# Event?
		if do_bounce == True:
			scoreOccurred = False
			# Do Event
			[vel_new, score_happened] = bounce_the_ball(ball_cmd_vel,last_bounce, left_hand_position, right_hand_position, ball_logic_position, paddle_size)

			#Publish hand twist
			vel_new_twist.linear.x=vel_new.x
			vel_new_twist.linear.y=vel_new.y
			hand_velocity_publisher.publish(vel_new_twist)

			# Score?
			if score_happened == True:
				# Incr score, return hand
				if last_bounce == 'lf':
					score[0] = score[0] + 1
				if last_bounce == 'rt':
					score[1] = score[1] + 1

				disp_score()
				last_bounce = 0
				# return hand to default position
				# TODO: hand_position_publisher move handd to center

		#Publish hand twist
		vel_new_twist.linear.x=vel_new.x
		vel_new_twist.linear.y=vel_new.y
		hand_velocity_publisher.publish(vel_new_twist)
		"""
		# Display
		pong_plot(bounds, plot_size, left_hand_position, right_hand_position, paddle_size, ball_logic_position)

		# If win, end game0[1]
		if np.max(score) >= max_score:
			disp_win(score)
			quit()

		# Update Rate Limiter
		r.sleep()

#########################################
# Boilerplate Code
if __name__ == '__main__':
	try:
		pong_logic()
	except rospy.ROSInterruptException:
		pass
