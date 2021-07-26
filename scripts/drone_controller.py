#!/usr/bin/env python	

import rospy
import sys
import math
import numpy as np
import tf
 

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from drone_swarm.srv import *
from drone_swarm.msg import *
from sensor_msgs.msg import LaserScan

import std_msgs


def set_movement_state(req):
	movement_state = req.movement_state
	return std_msgs.Empty()

#--------------------------------------------------------------------------------------------------------------------#
#                                             Parse Arguments                                                        #
#--------------------------------------------------------------------------------------------------------------------#
def ParseArgs(args, out):
	for arg in args:
		if ':=' in arg:
			temp = arg.split(':=')
			out[temp[0]] = temp[1]

arguments = {}
mandatory_keys = ['model']

ParseArgs(sys.argv, arguments)



# Make sure all the nessessary keys are added
if(any([not i in arguments.keys() for i in mandatory_keys])):
	raise ValueError('Not all mandatory keys are present. Check node arguments')


amount_of_drones = int(rospy.get_param('/amount_of_drones', "0"))

leader_drone = rospy.get_param('/leader_drone', "")

#--------------------------------------------------------------------------------------------------------------------#
#                                             Node Setup                                                             #
#--------------------------------------------------------------------------------------------------------------------#
rospy.init_node(f"drone_{arguments['model']}")
print(f'drone controller for model {arguments["model"]} has started')

# to get the position of the model
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model = GetModelStateRequest()
model.model_name = arguments['model']

# to set the state of the gazebo model
state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

# to publish the current position of this drone
position_pub = rospy.Publisher('position', DronePosition, queue_size=10)

# subscribe to the position of all other drones
drone_positions = {}
def set_drone_position(data):
	global drone_positions
	drone_positions[data.drone_name] = [data.x, data.y, data.z]

drone_position_sub =  []
for i in range(1, amount_of_drones + 1):
	if(str(i) == arguments['model'][-1]): continue
	drone_position_sub.append(rospy.Subscriber(f'/drone{i}/position', DronePosition, set_drone_position))

# set target
def set_target(data):
	global target_state
	
	target_state = [data.x_pos, data.y_pos, data.z_pos, data.x_ang, data.y_ang, data.z_ang]
	print(target_state)

set_target_sub = rospy.Subscriber('set_target', SetTarget, set_target)
#--------------------------------------------------------------------------------------------------------------------#
#                                                Vector Math                                                         #
#--------------------------------------------------------------------------------------------------------------------#

def ScaleVector(vector, newLength):
	length = math.sqrt(sum([i**2 for i in vector]))
	if length == 0: return [0 for i in vector]
	return [newLength*i/length for i in vector]

#--------------------------------------------------------------------------------------------------------------------#
#                                             PID controls Setup                                                     #
#--------------------------------------------------------------------------------------------------------------------#
P_GAIN = 5
I_GAIN = 0
D_GAIN = 5
 
PID = lambda e, el, et : P_GAIN * e + I_GAIN * et + D_GAIN * (e - el)
PD = lambda e, el: P_GAIN * e + D_GAIN * (e - el)
 
def ClampAcceleration(accel, max_acceleration):
	length = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
	
	if abs(length) > max_acceleration:
		
		return [max_acceleration*accel[0]/length, max_acceleration*accel[1]/length, max_acceleration*accel[2]/length]
	else:
		return accel
 
def set_pid(data):
	global P_GAIN
	global I_GAIN
	global D_GAIN

	P_GAIN = data.p
	I_GAIN = data.i
	D_GAIN = data.d
	
	print(f"{P_GAIN} {I_GAIN} {D_GAIN}")

set_pid_sub = rospy.Subscriber('set_pid', SetPID, set_pid)

last_error =  [0, 0, 0, 0, 0, 0]
total_error = [0, 0, 0, 0, 0, 0]
target_state = [0, 0, 10, 0, 0, 0]

def MoveToTargetState(current_state, target_state):
	global last_error
	global total_error
	
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
	new_state.pose.orientation.w = 1
	
	# twist
	accel = [PID(current_error[0], last_error[0], total_error[0]),
		 PID(current_error[1], last_error[1], total_error[1]),
		 PID(current_error[2], last_error[2], total_error[2])]
		 
	accel = ClampAcceleration(accel, 10)
	
	new_state.twist.linear.x += accel[0]
	new_state.twist.linear.y += accel[1]
	new_state.twist.linear.z += accel[2]
	#new_state.twist.angular.x += PID(current_error[3], last_error[3], total_error[3], 0.1)
	#new_state.twist.angular.y += PID(current_error[4], last_error[4], total_error[4], 0.1)
	#new_state.twist.angular.z += PID(current_error[5], last_error[5], total_error[5], 0.1)
	
	last_error = current_error.copy()
	
	return new_state
#--------------------------------------------------------------------------------------------------------------------#	
#                                              Pathing                                                               #
#--------------------------------------------------------------------------------------------------------------------#

path = [[0, 0, 15], [5, 4, 16], [10, 9, 20]]

#--------------------------------------------------------------------------------------------------------------------#	
#                                          Boids Algorithm                                                           #
#--------------------------------------------------------------------------------------------------------------------#


unit_vectors = []
unit_vectors_calculated = False


def OnBoidStart(data):
	global unit_vectors_calculated
	global unit_vectors
	
	print([data.angle_min + (data.angle_increment * i) for i in range(len(data.ranges))])
	for theta in [data.angle_min + (data.angle_increment * i) for i in range(len(data.ranges))]:
		unit_vectors.append([math.cos(theta), math.sin(theta)])
		print(unit_vectors[-1])
	
	
	unit_vectors_calculated = True

laser_data = LaserScan() 


def handle_laser(data):
	global laser_data
	if unit_vectors_calculated == False: OnBoidStart(data)
	
	laser_data = data
	
	
laser_sub = rospy.Subscriber('laser', LaserScan, handle_laser)




def AvoidObstacles(target_vector):
	global unit_vectors
	
	clear_casts = []
	
	start = None
	for i, distance in enumerate(laser_data.ranges):
		if not math.isinf(distance):
			if(start == None): 
				start = i
				print(f'start: {i}')
			else: 
				clear_casts.append([i for i in range(start+1, i)])
				print(f'end:{i}')
				start = None
			
	else:
		if(not start == None):
			clear_casts.append([i for i in range(start+1, i+1)])
			print(f'end:{i}')
			
		
				
			
	possible_paths = []		
	for group in clear_casts:
		if(len(group) > 12): 
			for path in group[6:-6]:
				possible_paths.append(path)
	
	#print(f'possible_paths: {len(possible_paths)}')
	
	closest_unit_vector = None
	
	closest_difference = float('inf')
		
	for path in possible_paths:	
		if closest_unit_vector == None:
			closest_unit_vector = unit_vectors[path]
			
		else:
			difference = math.acos(unit_vectors[path][0]*target_vector[0] + unit_vectors[path][1]*target_vector[1])
			
			if(difference < closest_difference):
				
				closest_unit_vector = unit_vectors[path]
				closest_difference = difference
				
	print(f'return: {closest_unit_vector} target_vector: {target_vector}')		
	return closest_unit_vector

def CalculateBoidVector(current_state, target_state):
	total_vector = [0.0, 0.0, 0.0]
	total_weight = 0
	
	
	# Attraction
	attraction_vector = ScaleVector([target_state[0]-current_state.pose.position.x,
			    	         target_state[1]-current_state.pose.position.y,
			                 target_state[2]-current_state.pose.position.z], 1)
		           	            
	
	total_vector[0] += attraction_vector[0] * 2
	total_vector[1] += attraction_vector[1] * 2
	total_vector[2] += attraction_vector[2] * 2
	
	total_weight += 2

	
	final_direction = AvoidObstacles(ScaleVector([component/total_weight for component in total_vector], 1)) 
	print(final_direction)
	return [final_direction[0] * 10, final_direction[1] * 10, (total_vector[2]/total_weight) * 10]
	
#--------------------------------------------------------------------------------------------------------------------#	
#                                             Main Loop                                                              #
#--------------------------------------------------------------------------------------------------------------------#

movement_state = "init_path"

def main():
	global movement_state
	global total_error
	global last_error
	global target_state
	
	

	r = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		current_state = get_model_srv(model)

		if movement_state == 'none':
			pass
		
		elif movement_state == "init_follow":
			total_error = [0, 0, 0, 0, 0, 0]
			last_error  = [0, 0, 0, 0, 0, 0]
			movement_state = "follow"
		
		elif movement_state == "follow":
		
			leader_pos = drone_positions[leader_drone]
			
			#print(f'Leader pos: {leader_pos}')
			#print(f'Target pos: {target_state}')
			
			
			relitive_target_state = [target_state[0] + leader_pos[0], 
						 target_state[1] + leader_pos[1], 
						 target_state[2] + leader_pos[2],
						 0, 0, 0, ]
			#print(f'Relitive target pos: {relitive_target_state}')			 
			state_pub.publish(MoveToTargetState(current_state, relitive_target_state))
		
		elif movement_state == "init_manual":
			total_error = [0, 0, 0, 0, 0, 0]
			last_error  = [0, 0, 0, 0, 0, 0]
			movement_state = 'manual'
		
		
		elif movement_state == 'manual':
			state_pub.publish(MoveToTargetState(current_state, target_state))
			
		elif movement_state == "traversing":
			velocity = CalculateBoidVector(current_state, target_state)
			
			new_state = ModelState()

			new_state.model_name = arguments['model']
			new_state.reference_frame = 'world'
	
			# pose
			new_state.pose = current_state.pose
			new_state.pose.orientation.x = 0
			new_state.pose.orientation.y = 0
			new_state.pose.orientation.z = 0
			new_state.pose.orientation.w = 1
			
			new_state.twist.linear.x = velocity[0]
			new_state.twist.linear.y = velocity[1]
			new_state.twist.linear.z = velocity[2]
			
			state_pub.publish(new_state)
		
		elif movement_state == "init_path":
			total_error = [0, 0, 0, 0, 0, 0]
			last_error  = [0, 0, 0, 0, 0, 0]
			movement_state = "path"
		
		elif movement_state == "path":
			global path
			
			if(len(path) == 0):
				print('path_complete')
				movement_state = 'manual'
				continue
				
			if(abs(path[0][0] - current_state.pose.position.x) < 0.5 and
			   abs(path[0][0] - current_state.pose.position.x) < 0.5 and
			   abs(path[0][0] - current_state.pose.position.x) < 0.5):
				target = path.pop(0)
				target_state = [target[0],
						target[1],
						target[2],
						0, 0, 0]
				
			if(len(path) > 0):
				state_pub.publish(MoveToTargetState(current_state, [path[0][0], path[0][1], path[0][2], 0, 0, 0]))
			
		else:
			print(f"{movement_state} is not a valid movement state" )

		
		drone_position = DronePosition()
		
		drone_position.drone_name = arguments['model']
		drone_position.x = current_state.pose.position.x
		drone_position.y = current_state.pose.position.y
		drone_position.z = current_state.pose.position.z
		
		position_pub.publish(drone_position)
		
		r.sleep()

#--------------------------------------------------------------------------------------------------------------------#
#                                             Apply Arguments                                                        #
#--------------------------------------------------------------------------------------------------------------------#
if 'target_pos' in arguments:
	temp = arguments['target_pos'].split(',')
	temp = [int(i) for i in temp]
	
	target_state[0] = temp[0]
	target_state[1] = temp[1]
	target_state[2] = temp[2]

#--------------------------------------------------------------------------------------------------------------------#
#                                             Commmand Handler                                                       #
#--------------------------------------------------------------------------------------------------------------------#

def handle_command(req):
	
	command = req.command.split()
	try:
		if(len(command) == 0): raise ValueError("no command specified")
		
		if(command[0] == 'update_params'):
			if(len(command) != 1): raise ValueError("command 'update_params' expects 0 arguments")
			
			global leader_drone
			
			leader_drone = rospy.get_param("/leader_drone", "")
	
	
		elif(command[0] == 'set_pid'):
			if(len(command) != 4): raise ValueError("command 'set_pid' expects 3 arguments (p_gain, i_gain, d_gain)")
		
			global P_GAIN
			global I_GAIN
			global D_GAIN

			P_GAIN = float(command[1])
			I_GAIN = float(command[2])
			D_GAIN = float(command[3])
			
			
		
		elif(command[0] == 'set_target'):
			if(len(command) != 7): raise ValueError("command 'set_target' expects 6 arguments (x, y, z, r, p y)")
			
			global target_state
			
			target_state = [float(command[1]),
					float(command[2]),
					float(command[3]),
					float(command[4]),
					float(command[5]),
					float(command[6])
					]
					

		elif(command[0] == 'set_movement_state'):
			if(len(command) != 2): raise ValueError("command 'set_movement_state' expects 1 arguments (new_movement_state)")
			
			global movement_state
			movement_state = command[1]
			
		elif(command[0] == 'query'):
			if(len(command) != 2): raise ValueError("command 'query' expects 1 arguments (value)")
			
			if(command[1] == 'pid_gains'):
				return CommandResponse(True, f"{P_GAIN} {I_GAIN} {D_GAIN}")
			
			elif(command[1] == 'target_pos'):
				print(f"True target state: {target_state[0]} {target_state[1]} {target_state[2]}")
				return CommandResponse(True, f"{target_state[0]} {target_state[1]} {target_state[2]}")
				
			elif(command[1] == 'movement_state'):
				return CommandResponse(True, movement_state)
				

			
		else:
			return CommandResponse(False, f"COMMAND FAILED: no command found '{command[0]}'")

		return CommandResponse(True, "COMMAND SUCCESFUL")
		
	except ValueError as e:
		return CommandResponse(False, f"COMMAND FAILED: {e}.")
	
command_service = rospy.Service('command', Command, handle_command)

#--------------------------------------------------------------------------------------------------------------------#	
main()
