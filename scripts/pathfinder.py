#!/usr/bin/env python	

import rospy
import sys
import math
import numpy as np
import tf
import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from drone_swarm.srv import *
from drone_swarm.msg import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import std_msgs

resolution = 0.2

'''
x_size = 100
y_size = 100
z_size = 100

scaler_field = [[[0 for i in range(x_size)] for i in range(y_size)] for i in range(z_size)]
'''
obstacles =[]
#--------------------------------------------------------------------------------------------------------------------#
#                                         Utility Functions                                                          #
#--------------------------------------------------------------------------------------------------------------------#

def WorldToGrid(point, snapped=True):
	'''
	Converts a point in world space to the algorithm's grid space.
	It's easier for the algorithm to work with int. 
	This converson lets that happen while retining some precision.

        Parameters
        ----------
        point: float[]
		the point in world space.
		[x, y, z]
		
	Key Word Args
	-------------
	snapped: bool
		if true the point will be snapped to the nearest whole number

        Returns
        -------
        float[]
        	the point in grid space
        '''
	if snapped:
		return [round(point[0]/resolution), round(point[1]/resolution), round(point[2]/resolution)]
		
	else:
		return [point[0]/resolution, point[1]/resolution, point[2]/resolution]	

def GridToWorld(point):
	'''
	Converts a point in grid space to world space.

        Parameters
        ----------
        point: float[] or Point
		the point in grid space.
		[x, y, z]
	
        Returns
        -------
        float[] or point
        	the point in world space
        	
        Notes
        -----
        The type of the output will be a float[] if point is a float[]
        and will be a Point if float is a Point.
        '''

	if isinstance(point, Point):
		return Point(point.x*resolution, point.y*resolution, point.z*resolution)
	return [point[0]*resolution, point[1]*resolution, point[2]*resolution]
#--------------------------------------------------------------------------------------------------------------------#
#                                          Pathfinding (a*)                                                          #
#--------------------------------------------------------------------------------------------------------------------#

#---------------------Helper-Node-Class--------------------#
class Node():
	def __init__(self, position, parent=None, is_start=False):
		'''
		Initializes a Node
	
		A Node is a point on a path that the pathfinding 
		algorithm will consider to.
		
		
        	Parameters
        	----------
        	position: float[]
			the position of the node on the grid
	
		
		Key Word Args
		-------------
		parent:
			the node before this one in its path
			
		is_start:
			indicates if this node is the start of a path
		
        	Returns
       	-------
       	None
       	'''
		self.is_start = is_start
	
		self.position = position
		self.parent = parent
		
		self.h = None
		self.g = 0 if is_start else math.inf
		self.f = 0
	
	def __eq__(self, other):
		'''
		Determines if two nodes are at the same position
		
        	Parameters
        	----------
        	other: Node
			the node to compare this one to
		
        	Returns
       	-------
       	bool
       		true if the nodes are at the same position
       	'''
		if(not isinstance(other, Node)): return False
		return self.position == other.position
	
	def calculate_heuristic(self, end):
		'''
		Calculates the distance to the end and 
		uses that as the h cost of the node.
		
		Parameters
		----------
		end: float[]
			the position of the end
			
		Returns
		-------
		None
       	'''
		self.h = math.dist(end, self.position) * 10
		
	def calculate_g(self, parent):
		'''
		Calculates cost of moving to this node from the parent node.
		This is used as the g cost. 
		
		Parameters
		----------
		parent: Node
			the parent node
			
		Returns
		-------
		None
		
		Warnings
		--------
		calculate_heuristic should be called first as this function
		will calculate the f cost after calculating the g cost
       	'''
		similar = 0
		if(parent.position[0] == self.position[0]): similar += 1
		if(parent.position[1] == self.position[1]): similar += 1
		if(parent.position[2] == self.position[2]): similar += 1
		
		
		
		new_g =  0.5 * parent.g + [17, 14, 10, 0][similar]
	
		#print(similar, new_g)
		
		if(new_g < self.g):
			self.g = new_g
			self.parent = parent

			self.f = self.g + self.h
			
#---------------------Helper-Functions---------------------#	

neighbour_increments = [[i, j, k] for i in [-1, 0, 1] for j in [-1, 0, 1] for k in [0]] 
	
neighbour_increments.remove([0, 0, 0])

def find_all_neighbours(point):
	'''
	Finds the positons of all possible neighbour nodes.
	
	Parameters
	----------
	point: float[]
		the position of the center point
		
	Returns
	-------
	float[][]
		a list of all the neighbour points
	'''
	return [[point[0] + neighbour_increment[0], 
		 point[1] + neighbour_increment[1], 
		 point[2] + neighbour_increment[2]] 
		 for neighbour_increment in neighbour_increments]
	
def find_node_with_lowest_f(nodes):
	'''
	Finds the node with the lowest f cost
	
	Parameters
	----------
	nodes: Node[]
		the nodes to search
		
	Returns
	-------
	int
		the index of the node with the lowest f cost
	'''

	lowest_index = 0
	lowest_value = math.inf	
	lowest_h = math.inf
	for i, node in enumerate(nodes):
		if node.f <= lowest_value:
			if(node.h < lowest_h):
				lowest_index = i
				lowest_value = node.f
				lowest_h = node.h
			
		
			
	return nodes[lowest_index]

#--------------------Pathing-Algorithm---------------------#

def find_path(start, end, display=False):
	'''
	Finds a path between two points using a* algorithm.
	
	Parameters
	----------
	start: float[]
		the point to start at
		
	end: float[]
		the point to end at
	Key Word Args
	-------------
	display
		if true the function will print what it is currently doing
	
	Returns
	-------
	Point[]
		list of all points in the path that was found
	'''
	
	
	startNode = Node([start[0], start[1], 0], is_start = True)
	startNode.calculate_heuristic(end)
	
	openNodes = [startNode]
	closedNodes = []
	
	endNode = None
	
	#----------find end node----------#
	
	while endNode == None:
		currentNode = find_node_with_lowest_f(openNodes)
		
		if(display): print(f'{currentNode.position} {currentNode.h} {currentNode.g} {currentNode.f}')
		
		#for node in openNodes:
		#	print(f'{node.position} {node.h} {node.g} {node.f}')
		
		openNodes.remove(currentNode)
		closedNodes.append(currentNode)
		
		if([currentNode.position[0], currentNode.position[1]] == [end[0], end[1]]):
			endNode = currentNode
			continue
			
		for neighbour in find_all_neighbours(currentNode.position):
			neighbourNode = Node(neighbour)
			if(neighbourNode.position[2] < 0): continue
			
			if (neighbourNode in closedNodes) or (neighbour in obstacles):
				continue
				
			if(neighbourNode in openNodes):
				openNodes[openNodes.index(neighbourNode)].calculate_g(currentNode)
			else:
				neighbourNode.calculate_heuristic(end)
				neighbourNode.calculate_g(currentNode)
				openNodes.append(neighbourNode)
		
				
	#----return all points in path----#
	
	# get all nodes in path
	path = []
	current_node = endNode
	
	while True:
		if(display): print(current_node.position)
		path.append(current_node.position)
		
		if(current_node.is_start): break
		
		current_node = current_node.parent
	
	# convert node into world position points
	out = [GridToWorld(Point(i[0], i[1], i[2])) for i in path]

		
	return out 
#--------------------------------------------------------------------------------------------------------------------#
#                                          Update Obstacles                                                          #
#--------------------------------------------------------------------------------------------------------------------#

def UpdateObstacles(data, drone_position):
	'''
	Uses laser data and drone position to find obstacles.
	
	Parameters
	----------
	data: LaserScan
		the laser data from a single drone
		
	drone_position: float[]
		the position of the drone

	Returns
	-------
	None
	'''
	
	for i, laser in list(enumerate(data.ranges))[::5]:
		if laser == float('inf'):
			continue
			#   if infinity (maybe add later) 
			#	check if the line intersects with any obstacles
			#	remove those obstacles
		else:
			obstacle_location = WorldToGrid( 
					    [
					     laser*unit_vectors[i][0] + drone_position[0], 
				  	     laser*unit_vectors[i][1] + drone_position[1], 
					     drone_position[2]
					    ],snapped=True)
			
			obstacle_locations = [[i, j, obstacle_location[2]] for i in range(obstacle_location[0]-1, obstacle_location[0]+2)
									    for j in range(obstacle_location[1]-3, obstacle_location[1]+4)]
			
			for obstacle in obstacle_locations:		     
				if(not obstacle in obstacles): 
					obstacles.append(obstacle)
					#print(f'obstacle found at {[obstacles[-1]]}')
				
	

#--------------------------------------------------------------------------------------------------------------------#
#                                             Node Setup                                                             #
#--------------------------------------------------------------------------------------------------------------------#

rospy.init_node(f"pathfinder")

amount_of_drones = int(rospy.get_param('/amount_of_drones', 0))

#-----------------Drone-Position-Subsriber-----------------#

drone_positions = {}
def set_drone_position(data):
	'''
	Stores the position of each drone.
	
	Parameters
	----------
	data: DronePosition
		the position of a drone along with its name

	Returns
	-------
	None
	'''
	global drone_positions

	drone_positions[data.drone_name] = [float(data.x), float(data.y), float(data.z)]
	
drone_position_sub =  []
for i in range(1, amount_of_drones + 1):
	drone_position_sub.append(rospy.Subscriber(f'/drone{i}/position', DronePosition, set_drone_position))

#------------------Drone-Laser-Subsriber-------------------#

# The unit vectors of each laser cast
unit_vectors = []


# Number of laser casts
samples = 0

laser_initialized = False

def OnLaserStart(data):
	'''
	Initializes values for using the laser data only called once.
 

        Parameters
        ----------
        data: LaserScan
        	The reading from the drone's laser sensor
		
        Returns
        -------
        None
        '''
	global laser_initialized
	global unit_vectors
	global samples
	
	# Calculate the unit vectors for each of the laser casts
	for theta in [data.angle_min + (data.angle_increment * i) for i in range(len(data.ranges))]:
		unit_vectors.append([math.cos(theta), math.sin(theta)])
	
	samples = len(unit_vectors)
	laser_initialized = True


def handle_laser(data):
	'''
	Stores the data from a drone's laser sensor.
	Initializes values for using the laser data.
 

        Parameters
        ----------
        data: LaserScan
        	The reading from a drone's laser sensor
		
        Returns
        -------
        None
        '''
	global laser_data
	if laser_initialized == False: OnLaserStart(data)
	
	if(data.header.frame_id[:6] in drone_positions):
		UpdateObstacles(data, drone_positions[data.header.frame_id[:6]])
	
laser_subs = []
for i in range(1, amount_of_drones + 1):
	laser_subs.append(rospy.Subscriber(f'drone{i}/laser', LaserScan, handle_laser))

#--------------------Find-Path-Service--------------------#

def find_path_handler(req):
	'''
	Handles find path requests
 
        Parameters
        ----------
        req: FindPathRequest
        	the request being made
		
        Returns
        -------
        FindPathResponse:
        	the path that was found
        '''
        
	out = FindPathResponse()
	
	if(len(req.point_a) != 3 or len(req.point_b) != 3):
		out.success = False
		return out

	out.path = find_path(WorldToGrid(list(req.point_a), snapped=True), WorldToGrid(list(req.point_b), snapped=True))
	
	out.path.reverse()

	out.success = True
	
	return out

pathfinder_service = rospy.Service('pathfinder/find_path', FindPath, find_path_handler)

#-----------------Is-Point-Clear-Service------------------#

def is_point_clear(req):
	'''
	Handles is point clear requests
 
        Parameters
        ----------
        req: IsPointClearRequest
        	the request being made
		
        Returns
        -------
        IsPointClearResponse:
        	whether or not the point is clear 
        '''
	out = IsPointClearResponse()
	out.is_point_clear = not WorldToGrid(req.point, snapped=True) in obstacles
	if(not out.is_point_clear): print(f'obstacle_found at {req.point}')
	return out


is_point_clear_service = rospy.Service('pathfinder/is_point_clear', IsPointClear, is_point_clear)


assert WorldToGrid([1, 1, 1], snapped=False) == [5, 5, 5], 'Test 1 Failed: (WorldToGrid with no snapping)'
assert WorldToGrid([2, 5, 10], snapped=False) == [10, 25, 50], 'Test 2 Failed: (WorldToGrid with no snapping)'
assert WorldToGrid([0, 0, 0], snapped=False) == [0, 0, 0], 'Test 3 Failed: (WorldToGrid with no snapping)'

assert WorldToGrid([1.25, 1.2, 1.1]) == [6, 6, 6], f'Test 4 Failed: (WorldToGrid with snapping) {WorldToGrid([1.25, 1.2, 1.1])}'


assert GridToWorld([5, 5, 5]) == [1, 1, 1], 'Test 5 Failed: (GridToWorld)'
assert GridToWorld([10, 25, 50]) == [2, 5, 10], 'Test 6 Failed: (GridToWorld)'
assert GridToWorld([0, 0, 0]) == [0, 0, 0], 'Test 7 Failed: (GridToWorld)'

#print(find_path([0, 0, 20], [0, 0, 0]))

rospy.spin()


