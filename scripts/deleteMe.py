import PySimpleGUI as sg


window_size = (1000, 500)


drone_position_info = lambda x, y, z: f'Drone Position: X:{round(x, 3)} Y:{round(y, 3)} Z:{round(z, 3)}'
drone_target_info = lambda x, y, z: f'Drone Target: X:{round(x, 3)} Y:{round(y, 3)} Z:{round(z, 3)}'

def WindowSize(x, y):
    return (round(window_size[0] * x), round(window_size[1] * y))
    
sg.theme('Dark')



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


tabs = sg.TabGroup([[sg.Tab("Manual Control", follow_control_tab),
		     sg.Tab("Follow Control", manual_control_tab)]], size=WindowSize(0.79, 0.53), enable_events = True, key='tabs')

info = sg.Frame("Drone Info", [[sg.Column([
					[sg.Text(drone_position_info(0, 0, 0))],
					[sg.Text(drone_target_info(0, 0, 0))]
					], size = WindowSize(1, 0.2))]])

drone_select = sg.Frame("Drone Select", [
			    [sg.Listbox([f'drone{i+1}' for i in range(2)], key='drone_select_1', size=(13, 14), enable_events=True)]
			    			      ])

layout = [[sg.Text("Ground Control", size = (50, 1), font=("Helvetica", 20), justification="center")],
	  [sg.Column([[tabs]], size=WindowSize(0.8, 0.6)), sg.Column([[drone_select]], size=WindowSize(0.2, 0.6))], 
	  [info]]

window = sg.Window("Ground Control", layout, size = window_size)


def main():
	print("main called")
	cont = True

	while cont:
		event, values = window.read()

		if(event == sg.WINDOW_CLOSED):
			print('window_closed')
			
			cont = False
        
	window.close()

print(sg.WIN_CLOSED == sg.WINDOW_CLOSED)
main()
