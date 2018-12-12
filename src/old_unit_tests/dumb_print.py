#!/usr/bin/env python
import sys
import glob
import pyfiglet

def runMe():
	result = pyfiglet.figlet_format("Pong", font = "drpepper")
	print(result)


if __name__ == '__main__':
    runMe()
