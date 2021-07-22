import rospy

def TryGetParam(param):
	print(rospy.get_param(param))
	print(f'Does {param} exist? {rospy.has_param(param)}')
	if(rospy.has_param(param)): 
		return [rospy.get_param(param), True]
	else: 
		return [None, False]

v, s = TryGetParam('"amount_of_drones"')
amount_of_drones = int(v) if(s) else 0 

print(amount_of_drones)
