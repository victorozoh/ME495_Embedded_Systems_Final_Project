#!/usr/bin/env python
#######################################
# Author: Petras Swissler
# Created: Dec 12, 2018
#######################################
# Import Required Libraries
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
