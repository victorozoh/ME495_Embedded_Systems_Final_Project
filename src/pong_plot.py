#!/usr/bin/env python
#######################################
# Author: Petras Swissler
# Created: Dec 12, 2018
#######################################

# This file contains a number of functions that combine to allow plotting of the pong game state in ASCII terminal output

#######################################
# Import Required Libraries
import  pyfiglet
from    pong_classes import *
import  sys
import  numpy as np

#######################################
# Global Variables
paddle_size_display_scale = 0.7

#######################################
# Helper Functions
def cprint(string_to_print):
    # Purpose:  implements simpler print behavior like C. THE WAY IT WAS MEANT TO BE
    # Inputs:   String
    # Outputs:  Prints string to the terminal

    sys.stdout.write(string_to_print)
    sys.stdout.flush()

def draw_floor_ceil(plot_size):
    # Purpose:  Draws the horizontal bars representing the floor and cieling of the arena
    # Inputs:   The width of the horizontal bars
    # Outputs:  Prints the horizontal bar to the terminal

    for col in range(plot_size.x):
        cprint("=")
    cprint("\n")
    return

def draw_paddle(row, bounds, plot_size, paddle_position, paddle_size):
    # Purpose:  Draws the paddle character '8' if a paddle is present. Otherwise, draws '-'
    # Inputs:   row is the current display row (int)
    #           bounds are the bounds of the arena (bound_lrud)
    #           plot_size is the dimensions of the display (xy_vector)
    #           paddle_position is the position of the center of the paddle
    #           paddle_size is the width of the paddle
    # Outputs:  Prints a character to the terminal

    # Flip the vertical row position to match mismatch between up-down draw and down-up logic
    vertical_pos = plot_size.y - row
    vert_domain = bounds.yhigh - bounds.ylow

    # Calculate paddle bounds in pixel space
    paddle_upper = (paddle_position + (paddle_size_display_scale*0.5 * paddle_size))*(plot_size.y/vert_domain)
    paddle_lower = (paddle_position - (paddle_size_display_scale*0.5 * paddle_size))*(plot_size.y/vert_domain)

    # Check if within paddle bounds
    if (vertical_pos <= paddle_upper)and(vertical_pos >= paddle_lower):
        cprint('8')
    else:
        cprint('-')
    return

def draw_game_line(row, bounds, plot_size, ball_position):
    # Purpose:  Draws a row of the game display
    # Inputs:   row is the current display row
    #           bounds are the bounds of the arena (bound_lrud)
    #           plot_size is the dimensions of the display (xy_vector)
    #           ball_position is the position of the center of the ball (xy_vector)
    # Outputs:  Draws the ball to the terminal using ASCII ccharacters

    # Display enlargening of the ball
    ballsize = 1.1

    # Flip the vertcal position of hte row
    vertical_pos = plot_size.y - row

    # Calculate the domain of arena
    vert_domain = bounds.yhigh - bounds.ylow
    horiz_domain = (bounds.xhigh - bounds.xlow)

    # Calculate ball position in pixel space
    ball_vert = (ball_position.y-bounds.ylow)*(plot_size.y/vert_domain)
    ball_horiz = (ball_position.x-bounds.xlow)*((plot_size.x-2)/horiz_domain)


    # Check if ball on this row
    if np.abs(ball_vert-vertical_pos) <= ballsize:
        # If in the row, then iterate through columns to check if to plot the ball
        for col in range(plot_size.x):
            if np.abs(ball_horiz-col) <= (ballsize):
                cprint("0")
            else:
                cprint(" ")
    else:
        # Otherwise, print spaces
        for col in range(plot_size.x):
            cprint(" ")

    return

def return_to_top(plot_size):
    # Purpose:  Returns the cursor to the top of the plot without overwriting to prevent scrolling
    # Inputs:   plot_size is the size of the plotted area (xy_vector)
    # Outputs:  none

    sys.stdout.write("\r")
    for ii in range(plot_size.y+4):
        sys.stdout.write("\033[F")


#######################################
# Main Function
def pong_plot(bounds, plot_size, left_paddle_position, right_paddle_position, paddle_size, ball_position):
    # Purpose:  Plots the entire pong field
    # Inputs:   bounds are the bounds of the arena (bound_lrud)
    #           plot_size is the dimensions of the display (xy_vector)
    #           left_paddle_position, right_paddle_postion is the position of the center of the paddle (float)
    #           paddle_size is the width of the paddle
    #           ball_position if the position of the ball (xy_vector)
    # Outputs:  Draws pong field to the terminal

    # Draw the header
    try: 
        pyfiglet.figlet_format("pong", font = "drpepper")
    except:
        pass

    # Draw Top Edge Bar
    draw_floor_ceil(plot_size)
    # Loop through all horizontal lines
    for row in range(plot_size.y+1):
        # Draw Paddle_left
        draw_paddle(row, bounds, plot_size, left_paddle_position, paddle_size)
        # Draw Line
        draw_game_line(row, bounds, plot_size, ball_position)
        # Draw_Paddle_right
        draw_paddle(row,bounds, plot_size, right_paddle_position, paddle_size)
        cprint("\n")
    # Draw Bot Edge Bar
    draw_floor_ceil(plot_size)

    return_to_top(plot_size)

#########################################
# Test Code
#if __name__ == '__main__':
#    try:
#        bound = bound_lrud(-0.0, 0.5, 0.6, 0.1)
#        plot_size = xy_vector(50,20)
#        left_paddle_position = 0.2
#        right_paddle_position = 0.4"\033[F"
#        paddle_size = 0.1
#        ball_position = xy_vector(.25,0.35)
#
#        pong_plot(bound, plot_size, left_paddle_position, right_paddle_position, paddle_size, ball_position)
#    except: #rospy.ROSInterruptException:
#        pass
