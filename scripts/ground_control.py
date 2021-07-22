#!/usr/bin/env python	

import PySimpleGUI as sg
import rospy
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from drone_swarm.srv import *
from drone_swarm.msg import *

### Globals ###

window_size = (1000, 500)

#--------------------------------------------------------------------------------------------------------------------#
#                                             Parse Arguments                                                        #
#--------------------------------------------------------------------------------------------------------------------#

def ParseArgs(args, out):
	for arg in args:
		if ':=' in arg:
			temp = arg.split(':=')
			out[temp[0]] = temp[1]
			

arguments = {}
ParseArgs(sys.argv, arguments)

mandatory_keys = ['amount_of_drones']

# Make sure all the nessessary keys are added
if(any([not i in arguments.keys() for i in mandatory_keys])):
	raise ValueError('Not all mandatory keys are present. Check node arguments')

amount_of_drones = int(rospy.get_param('/amount_of_drones'))

#--------------------------------------------------------------------------------------------------------------------#
#                                             Node Setup                                                             #
#--------------------------------------------------------------------------------------------------------------------#

rospy.init_node('ground_control')

#rospy.wait_for_service('/gazebo/get_model_state')
#get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


#command_clients = []

#for i in range(1, 1 + int(arguments['amount_of_drones'])):
#	command_clients.append(rospy.ServiceProxy(f'/drone{i}/command', Command))

# subscribe to the position of all other drones
drone_positions = {}
def set_drone_position(data):
	global drone_positions
	drone_positions[int(data.drone_name[-1])] = data

drone_position_sub =  []
for i in range(1, amount_of_drones + 1):
	drone_position_sub.append(rospy.Subscriber(f'/drone{i}/position', DronePosition, set_drone_position))


def SendCommand(drone_number, req):
	print(f'/drone{drone_number}/command', req)
	try:
		command_client = rospy.ServiceProxy(f'/drone{drone_number}/command', Command)
		res = command_client(req)
		return res.response
	
	except rospy.ServiceProxyException as e:
		return f'Command Failed: {e}'

#--------------------------------------------------------------------------------------------------------------------#	
#                                             GUI Layout                                                             #
#--------------------------------------------------------------------------------------------------------------------#

def WindowSize(x, y):
    return (round(window_size[0] * x), round(window_size[1] * y))
    
sg.theme('Dark')
manual_control_elements = [[
			    sg.Frame("Drone Select", [
			    [sg.Listbox([f'drone{i+1}' for i in range(2)], key='drone_select', size =(15, 19), enable_events=True)]
			    			      ], key='hrueq9y78g'),
			    sg.Column([
			    [sg.Text(' PID Gains:', size=(20, 1)), 
			     sg.Text('P:'),sg.Input(key='p_gain', size=(4, 1), enable_events=True),
			     sg.Text('I:'),sg.Input(key='i_gain', size=(4, 1), enable_events=True),
			     sg.Text('D:'),sg.Input(key='d_gain', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_pid_gains')],
			     
			    [sg.Text('Target Pos:', size=(20, 1)), 
			     sg.Text('X:'),sg.Input(key='x_target', size=(4, 1), enable_events=True),
			     sg.Text('Y:'),sg.Input(key='y_target', size=(4, 1), enable_events=True),
			     sg.Text('Z:'),sg.Input(key='z_target', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_target_pos')],
					
			    [sg.Text('Movement State:', size=(20, 1)),
			     sg.Input(key='new_state', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_movement_state')]
					])
			    ]]
manual_control_layout = [[sg.Column(manual_control_elements, size=WindowSize(1, 0.8))]]

follow_control_elements = [[
			    sg.Frame("Drone Select", [
			    [sg.Listbox([f'drone{i+1}' for i in range(2)], key='drone_select_1', size=(15, 19), enable_events=True)]
			    			      ]),
			     sg.Column([
			    [sg.Text('Target Pos:', size=(20, 1)), 
			     sg.Text('X:'),sg.Input(key='x_target_1', size=(4, 1), enable_events=True),
			     sg.Text('Y:'),sg.Input(key='y_target_1', size=(4, 1), enable_events=True),
			     sg.Text('Z:'),sg.Input(key='z_target_1', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_target_pos')],
					
			    [sg.Text('Set As Leader:', size=(20, 1)),
			     sg.Button('Set', key='set_as_leader')]
					])
			    ]]
			   
follow_control_layout = [[sg.Column(follow_control_elements, size=WindowSize(1, 0.8))]]

tabs = sg.TabGroup([[sg.Tab("Manual Control", manual_control_layout),
		     sg.Tab("Follow Control", follow_control_layout)]], enable_events = True, key='tabs')

layout = [[sg.Text("Ground Control", size = (50, 1), font=("Helvetica", 20), justification="center")],
	  [tabs]]

window = sg.Window("Ground Control", layout, size = window_size)


#--------------------------------------------------------------------------------------------------------------------#	
#                                             Main Loop                                                              #
#--------------------------------------------------------------------------------------------------------------------#

def AreAllValuesPresent(list, dict):
	return all([not dict[value] == "" for value in list]) 

def tryFloat(string):
	try:
		return float(string)
	except ValueError:
		return float(0)
		
def GetDronePosition(model_name):
	model = GetModelStateRequest()
	model.model_name = model_name
	
	drone_state = get_model_srv(model)
	return (drone_state.pose.position.x,
		drone_state.pose.position.y,
		drone_state.pose.position.z)
#--------------------------------------------------------------------------------------------------------------------#		
def main():
	selected_drone = ''

	cont = True
	#r = rospy.Rate(100)
	while (not rospy.is_shutdown()) and cont:
		event, values = window.read()
		
		event = str(event)
		
		
		if(event == sg.WINDOW_CLOSED):
        		cont = False

		elif(event == 'tabs'):
        		print(values['tabs'])
        		if(values['tabs'] == 'Manual Control'):
        			for i in range(amount_of_drones):
        				SendCommand(i+1, "update_params")
        			
        		elif(values['tabs'] == 'Follow Control'):
        			pass
        			
		elif(event == 'drone_select'):
			selected_drone = values["drone_select"][0] 
			print(f'Drone Selected: {selected_drone}')
		
		elif(event == 'drone_select_1'):
			selected_drone = values["drone_select_1"][0] 
			print(f'Drone Selected: {selected_drone}')
		
		elif(event == 'send_pid_gains'):
			if AreAllValuesPresent(['drone_select', 'p_gain', 'i_gain', 'd_gain'], values):

				SendCommand(int(selected_drone[-1]), f'set_pid {tryFloat(values["p_gain"])} {tryFloat(values["i_gain"])} {tryFloat(values["d_gain"])}')
				
			else:
				print('Ground Control: send_pid failed, not all values are present')	
				
		elif(event == 'send_target_pos'):
			if AreAllValuesPresent(['drone_select', 'x_target', 'y_target', 'z_target'], values):
				print(int(selected_drone[-1]))
				SendCommand(int(selected_drone[-1]), f'set_target {values["x_target"]} {values["y_target"]} {values["z_target"]} 0 0 0')
				
		
			else:	
				print('Ground Control: send_target failed, not all values are present')
		
		elif(event == 'send_movement_state'):
			if AreAllValuesPresent(['drone_select', 'new_state'], values):
				SendCommand(int(selected_drone[-1]), f'set_movement_state {values["new_state"]}')
		#r.sleep()
		
	window.close()


#--------------------------------------------------------------------------------------------------------------------#
if __name__ == '__main__':
	main()

