#!/usr/bin/env python3	

import rospy
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

# Parse arguments into a dict
arguments = {}
mandatory_keys = ['model']

# PID constants and variables

P_GAIN = 2
I_GAIN = 0.4
D_GAIN = 1

current_pose = [0, 0, 0, 0, 0, 0]

last_error = [0, 0, 0, 0, 0, 0]
current_error = [0, 0, 0, 0, 0, 0]

total_error = [0, 0, 0, 0, 0, 0]

target_state = [12, 15, 10, 1, 1, 1]

PID = lambda e, el, et, max_accel : max( min(P_GAIN * e + I_GAIN * et + D_GAIN * (e - el), max_accel), -1 * max_accel)

def ParseArgs(args, out):
	for arg in args:
		if ':=' in arg:
			temp = arg.split(':=')
			out[temp[0]] = temp[1]
	
ParseArgs(sys.argv, arguments)

# Make sure all the nessessary keys are added
if(any([not i in arguments.keys() for i in mandatory_keys])):
	raise ValueError('Not all mandatory keys are present. Check node arguments')

rospy.init_node("drone_controller")
print(f'drone controller for model {arguments["model"]} has started')

# Publisher Setup
state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState)

# Service setup
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model = GetModelStateRequest()
model.model_name = arguments['model']


print(f'drone controller for model {arguments["model"]} has made connection to service')

# Main loop
r = rospy.Rate(20)

while not rospy.is_shutdown():
	last_error = current_error.copy()
	
	current_state = get_model_srv(model)
	
	current_pose = [current_state.pose.position.x,
			current_state.pose.position.y,
			current_state.pose.position.z,
			current_state.pose.orientation.x,
			current_state.pose.orientation.y,
			current_state.pose.orientation.z]
	
	# calculate error
	current_error = [target_state[i] - current_pose[i] for i in range(6)]
		
	for i in range(6):
		total_error[i] += current_error[i]
	
	# apply accelerations
	new_state = ModelState()

	new_state.model_name = arguments['model']
	new_state.reference_frame = 'world'
	
	# pose
	new_state.pose = current_state.pose
	
	# twist
	new_state.twist.linear.x += PID(current_error[0], last_error[0], total_error[0], 3)
	new_state.twist.linear.y += PID(current_error[1], last_error[1], total_error[1], 3)
	new_state.twist.linear.z += PID(current_error[2], last_error[2], total_error[2], 3)
	new_state.twist.angular.x += PID(current_error[3], last_error[3], total_error[3], 0.1)
	new_state.twist.angular.y += PID(current_error[4], last_error[4], total_error[4], 0.1)
	new_state.twist.angular.z += PID(current_error[5], last_error[5], total_error[5], 0.1)
	
	state_pub.publish(new_state)

	r.sleep()
	
