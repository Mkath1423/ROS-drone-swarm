#!/usr/bin/env python	

import rospy
import sys
import numpy as np
 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
#from drone_swarm.srv import SetPID
 
# PID constants and variables

P_GAIN = 2
I_GAIN = 0.1
D_GAIN = 7
 
current_pose = [0, 0, 0, 0, 0, 0]

last_error = [0, 0, 0, 0, 0, 0]
current_error = [0, 0, 0, 0, 0, 0]

total_error = [0, 0, 0, 0, 0, 0]

target_state = [0, 0, 10, 0, 0, 0]

PID = lambda e, el, et, max_accel : max( min(P_GAIN * e + I_GAIN * et + D_GAIN * (e - el), max_accel), -1 * max_accel)
 
 # quaternion_rotation_matrix function by Addison Sers-Collins 
 # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def set_pid(req):
	P_GAIN = req.p
	I_GAIN = req.i
	D_GAIN = req.d


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

rospy.init_node("drone_controller")
print(f'drone controller for model {arguments["model"]} has started')

# Publisher Setup
state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState)

# Service setup
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model = GetModelStateRequest()
model.model_name = arguments['model']

#rospy.init_node('set_pid')
#s = rospy.Service('set_pid', SetPID, set_pid)
#rospy.spin()


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
	
	rot_matricx = quaternion_rotation_matrix([current_state.pose.orientation.w,
						   current_state.pose.orientation.x,
						   current_state.pose.orientation.y,
						   current_state.pose.orientation.z])
						   
	#print(f'{current_state.pose.orientation} -> {rot_matricx}')
	
	acceleration = np.array([PID(current_error[0], last_error[0], total_error[0], 10),
				 PID(current_error[1], last_error[1], total_error[1], 10),
				 PID(current_error[2], last_error[2], total_error[2], 10)])
	
	acceleration = np.dot(rot_matricx, acceleration)
	
	# twist
	new_state.twist.linear.x += PID(current_error[0], last_error[0], total_error[0], 10)
	new_state.twist.linear.y += PID(current_error[1], last_error[1], total_error[1], 10)
	new_state.twist.linear.z += PID(current_error[2], last_error[2], total_error[2], 10)
	new_state.twist.angular.x += PID(current_error[3], last_error[3], total_error[3], 0.1)
	new_state.twist.angular.y += PID(current_error[4], last_error[4], total_error[4], 0.1)
	new_state.twist.angular.z += PID(current_error[5], last_error[5], total_error[5], 0.1)
	
	state_pub.publish(new_state)

	r.sleep()
	
