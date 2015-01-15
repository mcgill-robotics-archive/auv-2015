#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot forward at a given velocity
"""
import action

class ForwardMovementAction(action.Action):
	desired_velocity = 0

	def __init__(self, my_autonomy, velocity):
		self.my_autonomy = my_autonomy
		self.desired_velocity = velocity
	
	def execute(self):	
		velocity_array = [self.desired_velocity, 0, 0, 0, 0, 0]
		self.my_autonomy.set_velocity(velocity_array)
		self.my_autonomy.print_info("forward movement action completed successfully")
		return True