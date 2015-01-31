#!/usr/bin/env python

import action

class SetSonarTargetAction(action.Action):
	desired_target = ""


	def __init__(self, my_autonomy, target):
		self.action_name = "Set Sonar Target"
		self.desired_target = target
		self.my_autonomy = my_autonomy


	def execute(self):
		self.print_start()
		self.my_autonomy.set_target(target)
		self.print_success()
		return True

	
