#!/usr/bin/env python

import action

class SetTransforms(action.Action):
	desired_position = 0

	def __init__(self, my_autonomy, position):
		self.action_name = "Get Position"
		self_desired_position = position
		self.my_autonomy = my_autonomy

	def execute(self):
		self.print_start()
			target_msg = position
		self.my_autonomy.set_se_transforms(target_msg)
		self.print_success()
		return True
