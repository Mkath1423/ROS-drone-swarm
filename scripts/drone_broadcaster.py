#!/usr/bin/env python	

import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import geometry_msgs

import tf2_ros
import sys

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



rospy.init_node('{arguments[model]}_broadcaster')

# Service to get drone position from
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

model = GetModelStateRequest()
model.model_name = arguments['model']

# Broadcaster setup
br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

t.header.frame_id = "world"
t.child_frame_id = arguments['model']

r = rospy.Rate(100)
def main():
	while not rospy.is_shutdown():
		current_state = get_model_srv(model)
	
		t.header.stamp = rospy.Time.now()
	
		t.transform.translation.x = current_state.pose.position.x
		t.transform.translation.y = current_state.pose.position.y
		t.transform.translation.z = current_state.pose.position.z
	
		t.transform.rotation = current_state.pose.orientation
	
		br.sendTransform(t)
	
		r.sleep()
		
main()

