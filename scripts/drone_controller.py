#!/usr/bin/env python	

import rospy
import sys
import numpy as np
import tf
 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from drone_swarm.srv import *
from drone_swarm.msg import *

# PID constants and variables

P_GAIN = 5
I_GAIN = 0
D_GAIN = 5
 
current_pose = [0, 0, 0, 0, 0, 0]

last_error = [0, 0, 0, 0, 0, 0]
current_error = [0, 0, 0, 0, 0, 0]

total_error = [0, 0, 0, 0, 0, 0]

target_state = [0, 0, 10, 0, 0, 0]

PID = lambda e, el, et, max_accel : max( min(P_GAIN * e + I_GAIN * et + D_GAIN * (e - el), max_accel), -1 * max_accel)
 
 
def set_pid(data):
	global P_GAIN
	global I_GAIN
	global D_GAIN

	P_GAIN = data.p
	I_GAIN = data.i
	D_GAIN = data.d
	
	print(f"{P_GAIN} {I_GAIN} {D_GAIN}")
	
def set_target(data):
	global target_state
	
	target_state = [data.x_pos, data.y_pos, data.z_pos, data.x_ang, data.y_ang, data.z_ang]
	print(target_state)

def ParseArgs(args, out):
	for arg in args:
		if ':=' in arg:
			temp = arg.split(':=')
			out[temp[0]] = temp[1]
			
# Parse arguments into a dict
arguments = {}
mandatory_keys = ['model']
ParseArgs(sys.argv, arguments)

# Make sure all the nessessary keys are added
if(any([not i in arguments.keys() for i in mandatory_keys])):
	raise ValueError('Not all mandatory keys are present. Check node arguments')

rospy.init_node(f"drone_{arguments['model']}")
print(f'drone controller for model {arguments["model"]} has started')

# Publisher Setup
state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState)

# Service setup
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model = GetModelStateRequest()
model.model_name = arguments['model']

set_pid_sub = rospy.Subscriber('set_pid', SetPID, set_pid)
set_target_sub = rospy.Subscriber('set_target', SetTarget, set_target)


print(f'drone controller for model {arguments["model"]} has made connection to service')

# Main loop
r = rospy.Rate(100)

while not rospy.is_shutdown():
	last_error = current_error.copy()
	
	current_state = get_model_srv(model)
	
	current_rotation = tf.transformations.euler_from_quaternion([current_state.pose.orientation.w,
						      current_state.pose.orientation.x,
						      current_state.pose.orientation.y,
						      current_state.pose.orientation.z])
	
	current_pose = [current_state.pose.position.x,
			current_state.pose.position.y,
			current_state.pose.position.z,
			current_rotation[0],
			current_rotation[1],
			current_rotation[2]]
	
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
	new_state.pose.orientation.x = 0
	new_state.pose.orientation.y = 0
	new_state.pose.orientation.z = 0
	new_state.pose.orientation.w = 0
	
	#rot_matricx = quaternion_rotation_matrix([current_state.pose.orientation.w,
	#					   current_state.pose.orientation.x,
	#					   current_state.pose.orientation.y,
	#					   current_state.pose.orientation.z])
						   
	#print(f'{current_state.pose.orientation} -> {rot_matricx}')
	
	#acceleration = np.array([PID(current_error[0], last_error[0], total_error[0], 10),
	#			 PID(current_error[1], last_error[1], total_error[1], 10),
	#			 PID(current_error[2], last_error[2], total_error[2], 10)])
	#
	#acceleration = np.dot(rot_matricx, acceleration)
	
	# twist
	new_state.twist.linear.x += PID(current_error[0], last_error[0], total_error[0], 10)
	new_state.twist.linear.y += PID(current_error[1], last_error[1], total_error[1], 10)
	new_state.twist.linear.z += PID(current_error[2], last_error[2], total_error[2], 10)
	# new_state.twist.angular.x += PID(current_error[3], last_error[3], total_error[3], 0.1)
	# new_state.twist.angular.y += PID(current_error[4], last_error[4], total_error[4], 0.1)
	#new_state.twist.angular.z += PID(current_error[5], last_error[5], total_error[5], 0.1)
	
	state_pub.publish(new_state)

	r.sleep()
	
