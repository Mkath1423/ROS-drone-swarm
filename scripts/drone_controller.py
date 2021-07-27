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
#                                                Vector Math                                                         #
#--------------------------------------------------------------------------------------------------------------------#

def VectorLength(vector):
	return math.sqrt(sum([i**2 for i in vector]))

def ScaleVector(vector, newLength):
	length = VectorLength(vector)
	if length == 0: return [0 for i in vector]
	return [newLength*i/length for i in vector]
	
def ClampVector(vector, max_magnitude):
	length = VectorLength(vector)
	
	if length > max_magnitude:
		return [max_magnitude*i/length for i in vector]
	else:
		return vector
		
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

def PublishAcceleration(accel, current_state):
	new_state = ModelState()

	new_state.model_name = arguments['model']
	new_state.reference_frame = 'world'
		
	new_state.pose = current_state.pose
	new_state.pose.orientation.x = 0
	new_state.pose.orientation.y = 0
	new_state.pose.orientation.z = 0
	new_state.pose.orientation.w = 1
		
	velocity = ClampVector([current_state.twist.linear.x + accel[0], 
				 current_state.twist.linear.y + accel[1],
				 current_state.twist.linear.z + accel[2]], 10)
	
	new_state.twist.linear.x = velocity[0]
	new_state.twist.linear.y = velocity[1]
	new_state.twist.linear.z = velocity[2]
		
	state_pub.publish(new_state)
# to publish the current position of this drone
position_pub = rospy.Publisher('position', DronePosition, queue_size=10)

# subscribe to the position of all other drones
drone_positions = {}
def set_drone_position(data):
	global drone_positions

	drone_positions[data.drone_name] = [float(data.x), float(data.y), float(data.z)]
	
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
#                                             PID controls Setup                                                     #
#--------------------------------------------------------------------------------------------------------------------#
P_GAIN = 5
I_GAIN = 0
D_GAIN = 5
 
def set_pid(data):
	global P_GAIN
	global I_GAIN
	global D_GAIN

	P_GAIN = data.p
	I_GAIN = data.i
	D_GAIN = data.d
	
	print(f"{P_GAIN} {I_GAIN} {D_GAIN}")

set_pid_sub = rospy.Subscriber('set_pid', SetPID, set_pid)
 
 
# PID = lambda e, el, et : P_GAIN * e + I_GAIN * et + D_GAIN * (e - el)
# PD = lambda e, el: P_GAIN * e + D_GAIN * (e - el)

class PIDController():
	def __init__(self, axes, P_GAIN=5, I_GAIN=0, D_GAIN=5):
		
		self.axes = axes
		
		self.P_GAIN = P_GAIN
		self.I_GAIN = I_GAIN
		self.D_GAIN = D_GAIN
		
		self.el = [0] * self.axes
		self.et = [0] * self.axes
		
	
		
	def Update(self, current_state, target_state):
		if(not len(current_state) == self.axes): 
			raise ValueError('PID Update Failed: not enough axes')
		
		# calculate current error
		e = [target_state[i] - current_state[i] for i in range(self.axes)]
		
		# calcualte the total error
		for i in range(self.axes):
			self.et[i] += e[i] 
		
		# calculate the output vector using PID function
		out = [P_GAIN * e[i] + I_GAIN * self.et[i] + D_GAIN * (e[i] - self.el[i]) for i in range(self.axes)]
		
		# update the last error
		self.el = e
		
		return out
 


target_state = [0, 0, 10]

#--------------------------------------------------------------------------------------------------------------------#	
#                                              Pathing                                                               #
#--------------------------------------------------------------------------------------------------------------------#

path = [[0, 0, 15], [5, 4, 16], [10, 9, 20]]
#--------------------------------------------------------------------------------------------------------------------#	
#                                          Obstacle Avoidance                                                        #
#--------------------------------------------------------------------------------------------------------------------#

# The unirt vectors of each laser cast
unit_vectors = []
unit_vectors_calculated = False

# Number of laser casts
samples = 0

def OnLaserStart(data):
	global unit_vectors_calculated
	global unit_vectors
	global samples
	
	# Calculate the unit vectors for each of the laser casts
	for theta in [data.angle_min + (data.angle_increment * i) for i in range(len(data.ranges))]:
		unit_vectors.append([math.cos(theta), math.sin(theta)])
		#print(unit_vectors[-1])
	
	samples = len(unit_vectors)
	unit_vectors_calculated = True

laser_data = LaserScan() 


def handle_laser(data):
	global laser_data
	if unit_vectors_calculated == False: OnLaserStart(data)
	
	laser_data = data
	
	
laser_sub = rospy.Subscriber('laser', LaserScan, handle_laser)

def AvoidObstacles(target_vector):
	global unit_vectors
	
	# store the length and direction of the target vector
	length = VectorLength(target_vector)
	target_direction = ScaleVector(target_vector, 1)
	
	
	# determine all the obstructed directions
	# store their indcies
	obstacles = []
	
	start = None
	for i, distance in enumerate(laser_data.ranges):
		if not math.isinf(distance):
			for j in range(i-30, i+31):
				obstacles.append(j % samples)
	
	obstacles = list(dict.fromkeys(mylist))
	
	# determine all non obstructed directions
	# store their incices
	possible_paths = [i for i in range(0, samples) if i not in obstacles]		
	
	
	# find the closes unobstructed direction to the target direction
	closest_path_unit_vector = None
	
	closest_path_difference = float('inf')
		
	for path in possible_paths:	
		if closest_path_unit_vector == None:
			closest_path_unit_vector = unit_vectors[path]
			closest_path_difference = math.acos(max(min(unit_vectors[path][0]*target_direction[0] + unit_vectors[path][1]*target_direction[1], 1), -1))
		else:
			difference = math.acos(max(min(unit_vectors[path][0]*target_direction[0] + unit_vectors[path][1]*target_direction[1], 1), -1))
			
			if(difference < closest_path_difference):
				closest_path_unit_vector = unit_vectors[path]
				closest_path_difference = difference
				
	closest_obstacle_unit_vector = None
	
	closest_obstacle_difference = float('inf')
	
	obstacle_index = 0
	
	for i, obstacle in enumerate(obstacles):
	
		if closest_obstacle_unit_vector == None:
			closest_obstacle_unit_vector = unit_vectors[obstacle]
			closest_obstacle_difference = math.acos(max(min(unit_vectors[obstacle][0]*target_direction[0] + unit_vectors[obstacle][1]*target_direction[1], 1), -1))
			obstacle_index = i
		else:
			difference = math.acos(max(min(unit_vectors[obstacle][0]*target_direction[0] + unit_vectors[obstacle][1]*target_direction[1], 1), -1))
			
			if(difference < closest_obstacle_difference):
				closest_obstacle_difference = unit_vectors[path]
				closest_obstacle_difference = difference
				obstacle_index = i
	
	length = 10 / laser_data.ranges[obstacle_index]
	
	# store the new direction
	out = []	
	if(closest_unit_vector == None): 
		 out = target_direction 
	else:
		out = closest_unit_vector + target_direction[2:]
		out = ScaleVector(out, 1)
	
	# Scale the vector to the original magnitude
	return [i * length for i in out]

#--------------------------------------------------------------------------------------------------------------------#	
#                                          Boids Algorithm                                                           #
#--------------------------------------------------------------------------------------------------------------------#

def CalculateBoidVector(current_state, target_state, attraction_gain=5, seperation_gain=6, cohesion_gain=1):
	current_position = [current_state.pose.position.x, 
			    current_state.pose.position.y, 
			    current_state.pose.position.z]
	
	total_vector = [0.0, 0.0, 0.0]
	total_weight = 0
	
	def add_to_total(vector, weight):
		total_vector[0] += vector[0] * weight
		total_vector[1] += vector[1] * weight
		total_vector[2] += vector[2] * weight
	
		nonlocal total_weight
		total_weight += weight
	
	
	# Attraction
	attraction_vector = ScaleVector([target_state[0]-current_position[0],
			    	         target_state[1]-current_position[1],
			                 target_state[2]-current_position[2]], 1)
		           	            
	
	add_to_total(attraction_vector, attraction_gain)

	average_position = [0, 0, 0]
	amount_of_drones = 0
	
	for drone in drone_positions.values():
		
		average_position[0] += drone[0]
		average_position[1] += drone[1]
		average_position[2] += drone[2]
		
		if math.dist(drone, current_position) < 2:
			add_to_total(ScaleVector([current_position[0]-drone[0], 
						  current_position[1]-drone[1], 
						  current_position[2]-drone[2]], 1), seperation_gain)
	
	
	
		amount_of_drones += 1
		
	average_position[0] /= amount_of_drones
	average_position[1] /= amount_of_drones
	average_position[2] /= amount_of_drones
	
	to_average_position = ScaleVector([average_position[0]-current_position[0],
			    	         average_position[1]-current_position[1],
			                 average_position[2]-current_position[2]], cohesion_gain)
	
	add_to_total(to_average_position, 1)
	
	return [total_vector[0]/total_weight, total_vector[1]/total_weight, total_vector[2]/total_weight]
	
#--------------------------------------------------------------------------------------------------------------------#	
#                                             Main Loop                                                              #
#--------------------------------------------------------------------------------------------------------------------#

movement_state = "init_manual"

def main():
	global movement_state
	global path
	global target_state
	

	r = rospy.Rate(50)
	
	while not rospy.is_shutdown():
	
		try:
			current_state = get_model_srv(model)
			
		except rospy.ServiceException as e:
			print(f'{arguments["model"]} failed to get model state ({rospy.Time.now()}): {e}')
			continue
			
		current_position = [current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z]
		current_velocity = [current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z]
		
		if movement_state == "init_follow":
			follow_PID = PIDController(3)
			movement_state = "follow"
		
		elif movement_state == "follow":
		
			leader_pos = drone_positions[leader_drone]
			
			relitive_target_state = [target_state[0] + leader_pos[0], 
						 target_state[1] + leader_pos[1], 
						 target_state[2] + leader_pos[2]]
						 
			accel = follow_PID.Update(current_position, relitive_target_state)

			accel = ClampVector(accel, 10)
			
			accel = AvoidObstacles(accel)
					 
			#distance = math.dist(relitive_target_state, current_position)
			 
			#scale = min(follow_PID.Update([0], [distance])[0], 10)
			
			#accel = [i * scale for i in direction]
			
			PublishAcceleration(accel, current_state)
		
		elif movement_state == "init_manual":
			manual_PID = PIDController(3)
			manual_accel_PID = PIDController(3)
			movement_state = 'manual'
		
		
		elif movement_state == 'manual':
			
			accel = manual_PID.Update(current_position, target_state)

			accel = ClampVector(accel, 10)

			PublishAcceleration(accel, current_state)
		
		elif movement_state == "init_traversing":
			traversing_PID = PIDController(1)
			movement_state = "traversing"
			
		elif movement_state == "traversing":
			direction = CalculateBoidVector(current_state, target_state)
			
			direction = AvoidObstacles(direction)
			
			distance = math.dist(target_state, current_position)
			
			scale = min(traversing_PID.Update([0], [distance])[0], 10)
			
			#print(direction)
			#print(scale)
			accel = [i * scale for i in direction]
			 
			PublishAcceleration(accel, current_state)
			
		
		elif movement_state == "init_path":
			path_PID = PIDController(3)
			target_state = path[0]
			movement_state = "path"
		
		elif movement_state == "path":
			
			if(len(path) == 0):
				print('path_complete')
				movement_state = 'init_manual'
				continue
				
			if(abs(path[0][0] - current_position[0]) < 0.5 and
			   abs(path[0][1] - current_position[1]) < 0.5 and
			   abs(path[0][2] - current_position[2]) < 0.5):
				target = path.pop(0)
				target_state = target
				
			if(len(path) > 0):
				accel = path_PID.Update(current_position, target_state)
					 
				accel = ClampVector(accel, 10)	
					 
				PublishAcceleration(accel, current_state)
			
		else:
			print(f"{movement_state} is not a valid movement state" )
			movement_state = 'init_manual'

		
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
			if(len(command) != 4): raise ValueError("command 'set_target' expects 3 arguments (x, y, z)")
			
			global target_state
			
			print(f'current target_state: {target_state}')
			
			target_state = [float(command[1]),
					float(command[2]),
					float(command[3])]
					
			print(f'recived target_state: {target_state}')
					

		elif(command[0] == 'set_movement_state'):
			if(len(command) != 2): raise ValueError("command 'set_movement_state' expects 1 arguments (new_movement_state)")
			
			global movement_state
			movement_state = 'init_' + command[1]
			
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
