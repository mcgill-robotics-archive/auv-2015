#! usr/bin/env python 

import task
import forward_movement_action
import set_cv_target

class PathTask(tast.Task:
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
			my_path_colour = set_cv_target.SetBallAction(self.my_autonomy, pathColor)
			my_position = set_se_transforms.SetTransforms(self.my_autonomy, position)
			my_forward_movement_action = forward_movement_action.ForwardMovementAction(self.my_autonomy, 3)
			self.action_stack.insert(0, my_path_colour)
			self.action_stack.insert(1, my_position)
			self.action_stack.insert(2, my_forward_movement_action)
