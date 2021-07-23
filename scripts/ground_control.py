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

### Parse arguments into a dict ###

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



### Setup node and ros functionalities ### 
rospy.init_node('ground_control')

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


set_pid_publishers = []
set_target_publishers = []

for i in range(1, 1 + int(arguments['amount_of_drones'])):
	set_pid_publishers.append(rospy.Publisher(f'/drone{i}/set_pid', SetPID, queue_size=10))
	set_target_publishers.append(rospy.Publisher(f'/drone{i}/set_target', SetTarget, queue_size=10))

<<<<<<< Updated upstream
### Functions ###
def GetDronePosition(model_name):
	model = GetModelStateRequest()
	model.model_name = model_name
	
	drone_state = get_model_srv(model)
	return (drone_state.pose.position.x,
		drone_state.pose.position.y,
		drone_state.pose.position.z)
	
=======
# subscribe to the position of all other drones
drone_positions = {}
def set_drone_position(data):
	global drone_positions
	drone_positions[data.drone_name] = [data.x, data.y, data.z]

drone_position_sub =  []
for i in range(1, amount_of_drones + 1):
	drone_position_sub.append(rospy.Subscriber(f'/drone{i}/position', DronePosition, set_drone_position))


def SendCommand(drone_name, req):
	print(f'/{drone_name}/command', req)
	try:
		command_client = rospy.ServiceProxy(f'/{drone_name}/command', Command)
		res = command_client(req)
		return res.response
	
	except rospy.service.ServiceException as e:
		return f'Command Failed: {e}'

#--------------------------------------------------------------------------------------------------------------------#	
#                                             GUI Layout                                                             #
#--------------------------------------------------------------------------------------------------------------------#

>>>>>>> Stashed changes
def WindowSize(x, y):
    return (round(window_size[0] * x), round(window_size[1] * y))
    
def AreAllValuesPresent(list, dict):
	return all([not dict[value] == "" for value in list])   
 
# Display a GUI window


# Main loop
#   Control GUI window
#   Control Drone movemnet by setting targets

sg.theme('Dark')
<<<<<<< Updated upstream
manual_control_elements = [[
			    sg.Frame("Drone Select", [
			    [sg.Listbox([f'drone{i+1}' for i in range(2)], key='drone_select', size =(15, 19), enable_events=True)]
			    			      ]),
			    sg.Column([
			    [sg.Text(' PID Gains:', size=(12, 1)), 
			     sg.Text('P:'),sg.Input(key='p_gain', size=(4, 1), enable_events=True),
			     sg.Text('I:'),sg.Input(key='i_gain', size=(4, 1), enable_events=True),
			     sg.Text('D:'),sg.Input(key='d_gain', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_pid_gains')],
			     
			    [sg.Text('Target Pos:', size=(12, 1)), 
			     sg.Text('X:'),sg.Input(key='x_target', size=(4, 1), enable_events=True),
			     sg.Text('Y:'),sg.Input(key='y_target', size=(4, 1), enable_events=True),
			     sg.Text('Z:'),sg.Input(key='z_target', size=(4, 1), enable_events=True),
			     sg.Button('Send', key='send_target_pos')]
					])
			    ]]
manual_control_layout = [[sg.Column(manual_control_elements, size=WindowSize(1, 0.8))]]

tabs = sg.TabGroup([[sg.Tab("Manual Control", manual_control_layout)]])
=======

drone_position_info = lambda x, y, z: f'Position: X:{round(x, 1)} Y:{round(y, 1)} Z:{round(z, 1)}'
drone_target_info = lambda x, y, z: f'Target: X:{round(x, 1)} Y:{round(y, 1)} Z:{round(z, 1)}'
drone_pid_info = lambda x, y, z: f'PID gains: P:{round(x, 1)} I:{round(y, 1)} D:{round(z, 1)}'
drone_movement_state_info = lambda state: f'Movement State: {state}'

manual_control_tab = [
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
		     ]

follow_control_tab = [ 
		      [sg.Text('Target Pos:', size=(20, 1)), 
		       sg.Text('X:'),sg.Input(key='x_target_1', size=(4, 1), enable_events=True),
		       sg.Text('Y:'),sg.Input(key='y_target_1', size=(4, 1), enable_events=True),
		       sg.Text('Z:'),sg.Input(key='z_target_1', size=(4, 1), enable_events=True),
		       sg.Button('Send', key='send_target_pos')],
				
		      [sg.Text('Set As Leader:', size=(20, 1)),
		       sg.Button('Set', key='set_as_leader')]	
		     ]


tabs = sg.TabGroup([[sg.Tab("Manual Control", manual_control_tab),
		     sg.Tab("Follow Control", follow_control_tab)]], size=WindowSize(0.79, 0.53), enable_events = True, key='tabs')

info = sg.Frame("Drone Info", [[sg.Column([
					    [sg.Text(drone_position_info(0, 0, 0), size = (40, 1), key = "drone_position_info"), sg.Text(drone_pid_info(0, 0, 0), size = (40, 1), key = "drone_pid_info")],
					    [sg.Text(drone_target_info(0, 0, 0), size = (40, 1), key = "drone_target_info"), sg.Text(drone_movement_state_info('manual'), size=(40,1), key='drone_movement_state_info')],
					    [sg.Button('Update', key='update_drone_info')]
					   ], size = WindowSize(1, 0.2))]])

drone_select = sg.Frame("Drone Select", [
			    [sg.Listbox([f'drone{i+1}' for i in range(2)], key='drone_select', size=(13, 14), enable_events=True)]
			    			      ])
>>>>>>> Stashed changes

layout = [[sg.Text("Ground Control", size = (50, 1), font=("Helvetica", 20), justification="center")],
	  [sg.Column([[tabs]], size=WindowSize(0.8, 0.6)), sg.Column([[drone_select]], size=WindowSize(0.2, 0.6))], 
	  [info]]

window = sg.Window("Ground Control", layout, size = window_size)

<<<<<<< Updated upstream
r = rospy.Rate(100)
def main():
	cont = True
	#while (not rospy.is_shutdown()) and cont:
	while cont:
=======

#--------------------------------------------------------------------------------------------------------------------#	
#                                             Main Loop                                                              #
#--------------------------------------------------------------------------------------------------------------------#

def AreAllValuesPresent(list, dict):
	for value in list:
		if(dict[value] == ""):
			print(f'{value} not in dict')
			return False
			
	return True

	#return all([not dict[value] == "" for value in list]) 

def TryFloat(string):
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
		                               
def UpdateDroneInfo(drone_name):
	drone_pos = drone_positions[drone_name]
	window.Element("drone_position_info").update(value=drone_position_info(drone_pos[0], drone_pos[1], drone_pos[2]))
	
	target_pos = SendCommand(drone_name, 'query target_pos').split()
	print(type(target_pos))
	print(f'recived target State: {target_pos[0]} {target_pos[1]}, {target_pos[2]}')
	print(f'after tryfloat: {TryFloat(target_pos[0])} {TryFloat(target_pos[1])} {TryFloat(target_pos[2])}')
	window.Element("drone_target_info").update(value=drone_target_info(TryFloat(target_pos[0]), TryFloat(target_pos[1]), TryFloat(target_pos[2])))
	window.Element("x_target").update(value=TryFloat(target_pos[0]))
	window.Element("y_target").update(value=TryFloat(target_pos[1]))
	window.Element("z_target").update(value=TryFloat(target_pos[2]))
	
	pid_gains = SendCommand(drone_name, 'query pid_gains').split()
	window.Element("drone_pid_info").update(value=drone_pid_info(TryFloat(pid_gains[0]), TryFloat(pid_gains[1]), TryFloat(pid_gains[2])))
	window.Element("p_gain").update(value=TryFloat(pid_gains[0]))
	window.Element("i_gain").update(value=TryFloat(pid_gains[1]))
	window.Element("d_gain").update(value=TryFloat(pid_gains[2]))
	
	
	
		                               
#--------------------------------------------------------------------------------------------------------------------#
	
def main():
	selected_drone = ''
	cont = True
	
	while (not rospy.is_shutdown()) and cont:
>>>>>>> Stashed changes
		event, values = window.read()
		
		if(event == sg.WINDOW_CLOSED):
        		cont = False
<<<<<<< Updated upstream
		
		elif(event == 'drone_select'):
			print(f'Drone Selected: {values["drone_select"][0]}')
		
		elif(event == 'send_pid_gains'):
			if AreAllValuesPresent(['drone_select', 'p_gain', 'i_gain', 'd_gain'], values):
			
				data = SetPID()
				
				data.p = int(''.join([ i for i in values['p_gain'] if i.isdecimal()]))
				data.i = int(''.join([ i for i in values['i_gain'] if i.isdecimal()]))
				data.d = int(''.join([ i for i in values['d_gain'] if i.isdecimal()]))
=======

		elif(event == 'tabs'):
        		print(values['tabs'])
        		if(values['tabs'] == 'Manual Control'):
        			for i in range(amount_of_drones):
        				SendCommand(f'drone{i+1}', "update_params")
        			
        		elif(values['tabs'] == 'Follow Control'):
        			pass
	
		elif(event == 'drone_select'):
			selected_drone = values["drone_select"][0]
			
			UpdateDroneInfo(selected_drone)
			print(f'Drone Selected: {selected_drone}')
		
		elif(event == 'update_drone_info'):
        		if(selected_drone == ''): continue
        		UpdateDroneInfo(selected_drone)
		
		elif(event == 'send_pid_gains'):
			if AreAllValuesPresent(['p_gain', 'i_gain', 'd_gain'], values):

				SendCommand(selected_drone, f'set_pid {TryFloat(values["p_gain"])} {TryFloat(values["i_gain"])} {TryFloat(values["d_gain"])}')
>>>>>>> Stashed changes
				
				set_pid_publishers[int(values['drone_select'][0][-1]) - 1].publish(data)
			else:	
				print('Ground Control: send_PID failed, not all values are present')	
				
		elif(event == 'send_target_pos'):
<<<<<<< Updated upstream
			if AreAllValuesPresent(['drone_select', 'x_target', 'y_target', 'z_target'], values):
			
				data = SetTarget()
=======
			if AreAllValuesPresent(['y_target', 'x_target', 'z_target'], values):
				print(values["x_target"])
				SendCommand(selected_drone, f'set_target {values["x_target"]} {values["y_target"]} {values["z_target"]} 0 0 0')
>>>>>>> Stashed changes
				
				data.x_pos = int(''.join([i for i in values['x_target'] if i.isdecimal()]))
				data.y_pos = int(''.join([i for i in values['y_target'] if i.isdecimal()]))
				data.z_pos = int(''.join([i for i in values['z_target'] if i.isdecimal()]))
				
				data.x_ang = 0
				data.y_ang = 0
				data.z_ang = 0
				
				set_target_publishers[int(values['drone_select'][0][-1]) - 1].publish(data)
			else:	
<<<<<<< Updated upstream
				print('Ground Control: send_PID failed, not all values are present')
		r.sleep()
=======
				print('Ground Control: send_target failed, not all values are present')
		
		elif(event == 'send_movement_state'):
			if AreAllValuesPresent(['drone_select', 'new_state'], values):
				SendCommand(selected_drone, f'set_movement_state {values["new_state"]}')
			
			else:	
				print('Ground Control: set_movement_state failed, not all values are present')
>>>>>>> Stashed changes
		
	window.close()



if __name__ == '__main__':
	main()

