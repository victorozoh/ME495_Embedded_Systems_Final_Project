#!/usr/bin/env python
#######################################
# Author: Petras Swissler
# Created: Dec 11, 2018
#######################################
# List of To-Dos:
    # None
# List of Bugs:
    # None
# List of Future Work:
    # Smooth the motion of the ball
    # Get pyfiglet working; crashes program when it tries to print to the screen. Pyfiglet would print wordart to the terminal

#######################################
# Import Required Libraries
# Standad and ROS imports
import 	rospy
import 	numpy as np
from intera_interface import Limb

# Project-specific imports
import hand_interface as hif
#import 	pyfiglet
from pong_plot import *

# Messages
from sawyer_pong.msg import measured_distances
from geometry_msgs.msg import (
    Twist,
    Pose
    )

#######################################
# Create Classes
class bound_lrud:
    # This class packages the boundary information into one simple-to-access object
    def __init__(self, xlow, xhigh, yhigh, ylow):
        self.xlow = xlow
        self.xhigh = xhigh
        self.yhigh = yhigh
        self.ylow = ylow

class xy_vector:
    # This class simply packages 2-dimensional vectors into one simple-to-access object
    def __init__(self, x, y):
        self.x = x
        self.y = y

#######################################
# Global and Configuration Variables 
# Positions of the player hands
hand_positions = [50,50]
# Position of the ball
ball_positions = xy_vector(50,50)
ball_measured_position = xy_vector(0,0)
# Logic locations of the ball
expected_position=xy_vector(0,0)
combined=xy_vector(0,0)
# Ball command velocities
vel_new=xy_vector(0,0)
# Display configuration
plot_size = xy_vector(200,50)

#######################################
# Helper Functions

def disp_score(score):
    # Purpose:  Displays the score after a point is scored
    # Inputs:   The score as a [player 1 score, player 2 score] array
    # Outputs:  Prints the score to the terminal

    #pyfiglet.figlet_format('SCORE', font = "drpepper")
    print(score[0]," to ",score[1])
    return 1


def disp_win(score):
    # Purpose:  Displays the winning player's information
    # Inputs:   The score as a [player 1 score, player 2 score] array
    # Outputs:  Prints winning player information to the termial

    if score[0] > score[1]:
        #print(" LEFT PLAYER WINS ")
        pyfiglet.figlet('LEFT PLAYER WINS')
    else:
        #print(" RIGHT PLAYER WINS ")
        pyfiglet.figlet_format('RIGHT PLAYER WINS')

####
def rand_sign():
    # Purpose:  Generates a random sign with 50/50 odds
    # Inputs:   None
    # Outputs:  Either [-1] or [1]

    choose = np.random.rand()
    if choose >= 0.5:
        return 1
    else:
        return -1
##
def get_ball_start_velocity(ball_speed):
    # Purpose:  Generates a random start velocity for the ball to take at the start of the game or after a point is scored
    # Inputs:   Desired ball speed (scalar value)
    # Outputs:  Initial ball velocity (xy_vector object)

    xx = (np.random.rand()*0.8 + 0.2)*rand_sign()
    yy = (np.random.rand()*0.4 + 0.0)*rand_sign()

    norm = np.sqrt(xx*xx + yy*yy)
    xx = ball_speed * xx / norm
    yy = ball_speed * yy / norm

    start_vel = xy_vector(xx,yy)
    return start_vel
####
def ball_expected_position(ball_logic_position, ball_cmd_vel, delta_time):
    # Purpose:  UNUSED: predict the ball position given necessary information
    # Inputs:   ball_logic_position is the last known position of the ball (xy_vector)
    #           ball_cmd_vel is the velocity of the ball (xy_vector)
    #           delta_time is the time since the last measurement (float)
    # Outputs:  The expected ball position (xy_vector)

    expected_position.x = ball_logic_position.x + ball_cmd_vel.x * delta_time
    expected_position.y = ball_logic_position.y + ball_cmd_vel.y * delta_time
    return expected_position
####
def combine_ball_positions(simulated, actual):
    # Purpose:  UNUSED Calculates a weighted average between the simulated and actual ball positions
    # Inputs:   Simulated ball position (xy_vector)
    #           Actual (measured) ball position (xy_vector)
    # Outputs:  Combined ball position (xy_vector)

    simweight = 0.8
    combined.x = simulated.x*simweight + actual.x*(1-simweight)
    combined.y = simulated.y*simweight + actual.y*(1-simweight)
    return combined
####
def check_bounds(position, bound, last_bounce):
    # Purpose:  Checks to see if the ball has reached any of the bounds
    # Inputs:   position is the position of the ball (xy_vector)
    #           bound is the boundaries of the game arena (bound_lrud)
    #           last_bounce is the boundary off which the last bounce occurred
    # Outputs:  do_bounce signals whether a bounce should be executed
    #           new_bounce is the updated version of last_bounce

    # Init for later logic
    new_bounce = last_bounce

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

#######################################
# Callbacks
def hand_position_callback(latest_msg):
    # Purpose:  Interfaces the main pong function with the 
    # Inputs:   latest_msg (measured_distance message)
    # Outputs:  Updates values of hand_positions global variable

    hand_positions[0] = latest_msg.left_distance
    hand_positions[1] = latest_msg.right_distance


def checkPosition(posemsg):
    # Purpose:  Checks the actual position of the pong ball
    # Inputs:   posemsg (Pose message)
    # Outputs:  Updates the values of ball_measured_postion global variables

    ball_measured_position.x = posemsg.position.x
    ball_measured_position.y = posemsg.position.z

#########################################
# Primary function
def pong_logic():
    # Purpose:  Executes the main pong logic behavior (i.e. all game code)
    # Inputs:   See parameters and subscribers below
    # Outputs:  Moves robot arm, displays the game state. See publishers below

    #Initialize the _node
    rospy.init_node('pong_Master')

    # define initial joint positions
    thetalistHOME = [1.3395771484375, -3.5355, 2.0365224609375, -1.489580078125, -0.4218515625, 1.1975029296875, -3.419748046875, 0.0]
    hif.move_to_home(thetalistHOME)

    #######################
    # Get parameters
    # UNUSED english_amount     = rospy.get_param('~english_amount',0)
    paddle_size        = rospy.get_param('~paddle_size',0.2)
    # UNUSED ball_velocity_incr = rospy.get_param('~ball_velocity_incr',1)
    ball_speed      = rospy.get_param('~ball_velocity',0.1)
    max_score          = rospy.get_param('~max_score',5)

    circle_tick_rate = rospy.get_param('~circle_tick_rate', 150)

    #######################
    # Create Publishers
    hand_velocity_publisher = rospy.Publisher('pongvelocity', Twist, queue_size=1)

    #######################
    # Create Subscribers
    # position subscriber:
    poscheck = rospy.Subscriber("endpoint_Pose", Pose, checkPosition, queue_size=1)
    # hand position subscriber:
    hand_position_subscriber = rospy.Subscriber('hand_positions', measured_distances, hand_position_callback,queue_size=1)

    #######################
    # Create the message variables
    veltwist=Twist()

    #######################
    # Create start variables
    score           = [0,0]
    ball_cmd_vel    = get_ball_start_velocity(ball_speed)

    veltwist.linear.x=ball_cmd_vel.x
    veltwist.linear.z=ball_cmd_vel.y

    left_hand_position  = hand_positions[0]/1000.0
    right_hand_position = hand_positions[1]/1000.0

    last_bounce = 0
    
    ball_logic_position = ball_measured_position 
    bounds = bound_lrud(-0.275,0.42,0.44,0.14)

    #######################
    # Perform time setup
    r = rospy.Rate(circle_tick_rate)
    start_time = rospy.get_time()
    now_time = start_time
    
    #######################
    # Main Loop
    while not rospy.is_shutdown():

        # Time Calcs
        last_time   = now_time
        now_time    = rospy.get_time() - start_time
        delta_time  = now_time - last_time

        # Calculate position to use for logic
        ball_logic_position = ball_measured_position #combine_ball_positions(ball_expected_position(ball_logic_position, ball_cmd_vel, delta_time), ball_recieved_position)

        # Get Hand Positions in meters
        left_hand_position  = hand_positions[0]/1000.0
        right_hand_position = hand_positions[1]/1000.0

        # Display
        print(ball_measured_position.y, left_hand_position)
        ###print(right_hand_position, left_hand_position)
        pong_plot(bounds, plot_size, left_hand_position, right_hand_position, paddle_size, ball_logic_position)

        # Check Bounds
        [do_bounce,last_bounce] = check_bounds(ball_logic_position, bounds, last_bounce)
        # Event?
        if do_bounce == True:
            score_happened = False
            if last_bounce == 'lf':
                veltwist.linear.x = np.abs(veltwist.linear.x)
                if np.abs(ball_measured_position.y-bounds.ylow-left_hand_position) > (paddle_size/2):
                    score_happened = True
                    print('miss:' + str(score_happened))
                if np.abs(ball_measured_position.y-bounds.ylow-left_hand_position) <= (paddle_size/2):
                    score_happened = False
                    print('recovery:' + str(score_happened))
            elif last_bounce == 'rt':
                veltwist.linear.x = -np.abs(veltwist.linear.x)
                if np.abs(ball_measured_position.y-bounds.ylow-right_hand_position) > (paddle_size/2):
                    score_happened = True
                    print('miss:' + str(score_happened))
                if np.abs(ball_measured_position.y-bounds.ylow-right_hand_position) <= (paddle_size/2):
                    score_happened = False
                    print('recovery:' + str(score_happened))
            if last_bounce == 'dn':
                veltwist.linear.z = np.abs(veltwist.linear.z)
            elif last_bounce == 'up':
                veltwist.linear.z = -np.abs(veltwist.linear.z)

            # Score?
            if score_happened == True:

                # Incr score, return hand, and pause
                if last_bounce == 'lf':
                    score[0] = score[0] + 1
                if last_bounce == 'rt':
                    score[1] = score[1] + 1

                veltwist.linear.x=0
                veltwist.linear.z=0
                hand_velocity_publisher.publish(veltwist)

                disp_score(score)
                last_bounce = 0
                # return hand to default position
                hif.move_to_home(thetalistHOME)

                ball_cmd_vel    = get_ball_start_velocity(ball_speed)

                veltwist.linear.x=ball_cmd_vel.x
                veltwist.linear.z=ball_cmd_vel.y
                # TODO: increase speed

        #Publish hand twist
        hand_velocity_publisher.publish(veltwist)


        # If win, end game
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
