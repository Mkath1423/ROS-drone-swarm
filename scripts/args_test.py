#!/usr/bin/env python3	

import rospy
import sys

arguments = {}

def ParseArgs(args, out):
	for arg in args:
		if ':=' in arg:
			temp = arg.split(':=')
			out[temp[0]] = temp[1]

ParseArgs(sys.argv, arguments)

print(sys.argv)
print("Argument List:", str(arguments))

