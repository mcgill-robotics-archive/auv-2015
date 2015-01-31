#!/usr/bin/env python 

import task
import x_axis_movement_action
import set_cv_target_action
import get_transform_action
 
class PathTask(task.Task):
	def __init__(self, phase, my_autonomy):
		self.task_name = "Path"
		self.phase = phase
		self.my_autonomy = my_autonomy
		self.load_action_stack()
	
	def __init__(self, my_autonomy):
		self.task_name = "Path"
		self.phase = 1
		self.my_autonomy = my_autonomy
		self.load_action_stack()
		
	def load_action_stack(self):
		if (self.phase > 0):
			my_path_colour = set_cv_target_action.SetCVTargetAction(self.my_autonomy, new_cv_target)
			my_position = get_transform_action.GetTransformAction(self.my_autonomy, new_target_frame)
			my_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 3)
			self.action_stack.insert(0, my_path_colour)
			self.action_stack.insert(0, my_position)
			self.action_stack.insert(0, my_x_axis_movement_action)
