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

### Functions ###
def GetDronePosition(model_name):
	model = GetModelStateRequest()
	model.model_name = model_name
	
	drone_state = get_model_srv(model)
	return (drone_state.pose.position.x,
		drone_state.pose.position.y,
		drone_state.pose.position.z)
	
def WindowSize(x, y):
    return (round(window_size[0] * x), round(window_size[1] * y))
    
def AreAllValuesPresent(list, dict):
	return all([not dict[value] == "" for value in list])   
 
# Display a GUI window


# Main loop
#   Control GUI window
#   Control Drone movemnet by setting targets

sg.theme('Dark')
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

layout = [[sg.Text("Ground Control", size = (50, 1), font=("Helvetica", 20), justification="center")],
	  [tabs]]

window = sg.Window("Ground Control", layout, size = window_size)

r = rospy.Rate(100)
def main():
	cont = True
	#while (not rospy.is_shutdown()) and cont:
	while cont:
		event, values = window.read()
		
		if(event == sg.WINDOW_CLOSED):
        		cont = False
		
		elif(event == 'drone_select'):
			print(f'Drone Selected: {values["drone_select"][0]}')
		
		elif(event == 'send_pid_gains'):
			if AreAllValuesPresent(['drone_select', 'p_gain', 'i_gain', 'd_gain'], values):
			
				data = SetPID()
				
				data.p = int(''.join([ i for i in values['p_gain'] if i.isdecimal()]))
				data.i = int(''.join([ i for i in values['i_gain'] if i.isdecimal()]))
				data.d = int(''.join([ i for i in values['d_gain'] if i.isdecimal()]))
				
				set_pid_publishers[int(values['drone_select'][0][-1]) - 1].publish(data)
			else:	
				print('Ground Control: send_PID failed, not all values are present')	
				
		elif(event == 'send_target_pos'):
			if AreAllValuesPresent(['drone_select', 'x_target', 'y_target', 'z_target'], values):
			
				data = SetTarget()
				
				data.x_pos = int(''.join([i for i in values['x_target'] if i.isdecimal()]))
				data.y_pos = int(''.join([i for i in values['y_target'] if i.isdecimal()]))
				data.z_pos = int(''.join([i for i in values['z_target'] if i.isdecimal()]))
				
				data.x_ang = 0
				data.y_ang = 0
				data.z_ang = 0
				
				set_target_publishers[int(values['drone_select'][0][-1]) - 1].publish(data)
			else:	
				print('Ground Control: send_PID failed, not all values are present')
		r.sleep()
		
	window.close()



if __name__ == '__main__':
	main()

