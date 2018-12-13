#!/usr/bin/env python
#######################################
# Author: Petras Swissler
# Created: Dec 12, 2018
#######################################
# Import Required Libraries
import     pyfiglet
from pong_classes import *
import sys
import numpy as np

#######################################
# Global Variables
paddle_size_display_scale = 0.7

#######################################
# Helper Functions
def cprint(string_to_print):
    sys.stdout.write(string_to_print)
    sys.stdout.flush()

def draw_floor_ceil(plot_size):
    for col in range(plot_size.x):
        cprint("=")
    cprint("\n")
    return

def draw_paddle(row, bounds, plot_size, paddle_position, paddle_size):

    vertical_pos = plot_size.y - row
    vert_domain = bounds.yhigh - bounds.ylow

    paddle_upper = (paddle_position + (paddle_size_display_scale*0.5 * paddle_size))*(plot_size.y/vert_domain)
    paddle_lower = (paddle_position - (paddle_size_display_scale*0.5 * paddle_size))*(plot_size.y/vert_domain)

    if (vertical_pos <= paddle_upper)and(vertical_pos >= paddle_lower):
        cprint('8')
    else:
        cprint('-')

    return

def draw_game_line(row, bounds, plot_size, ball_position):
    ballsize = 1.1

    vertical_pos = plot_size.y - row

    vert_domain = bounds.yhigh - bounds.ylow
    horiz_domain = (bounds.xhigh - bounds.xlow)


    ball_vert = (ball_position.y-bounds.ylow)*(plot_size.y/vert_domain)
    ball_horiz = (ball_position.x-bounds.xlow)*((plot_size.x-2)/horiz_domain)


    # Check if ball on this row
    if np.abs(ball_vert-vertical_pos) <= ballsize:
        for col in range(plot_size.x):
            if np.abs(ball_horiz-col) <= (ballsize):
                cprint("0")
            else:
                cprint(" ")
    else:
        for col in range(plot_size.x):
            cprint(" ")

    return

def return_to_top(plot_size):
    sys.stdout.write("\r")
    for ii in range(plot_size.y+4):
        sys.stdout.write("\033[F")


#######################################
# Main Function
def pong_plot(bounds, plot_size, left_paddle_position, right_paddle_position, paddle_size, ball_position):
    # Draw the header
    pyfiglet.figlet_format("pong", font = "drpepper")

    # Draw Top Edge Bar
    draw_floor_ceil(plot_size)
    # Loop through all horizontal lines
    for row in range(plot_size.y+1):
        #print('hi')
        # Draw Paddle_leftpaddle_size
        draw_paddle(row, bounds, plot_size, left_paddle_position, paddle_size)
        # Draw Line
        draw_game_line(row, bounds, plot_size, ball_position)
        # Draw_Paddle_right
        draw_paddle(row,bounds, plot_size, right_paddle_position, paddle_size)
        cprint("\n")
    # Draw Bot Edge Bar
    #print("hi")
    draw_floor_ceil(plot_size)

    return_to_top(plot_size)

#########################################
# Boilerplate Code
"""if __name__ == '__main__':
    try:
        bound = bound_lrud(-0.0, 0.5, 0.6, 0.1)
        plot_size = xy_vector(50,20)
        left_paddle_position = 0.2
        right_paddle_position = 0.4"\033[F"
        paddle_size = 0.1
        ball_position = xy_vector(.25,0.35)

        pong_plot(bound, plot_size, left_paddle_position, right_paddle_position, paddle_size, ball_position)
    except: #rospy.ROSInterruptException:
        pass"""
