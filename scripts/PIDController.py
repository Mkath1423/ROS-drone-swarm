#!/usr/bin/env python	

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
			
		e = [target_state[i] - current_state[i] for i in range(self.axes)]
		
		for i in range(self.axes):
			self.et[i] += e[i] 
		
		
		out = [P_GAIN * e + I_GAIN * et + D_GAIN * (e - el) for i in range(axes)]
		
		self.el = e
		
		return out
