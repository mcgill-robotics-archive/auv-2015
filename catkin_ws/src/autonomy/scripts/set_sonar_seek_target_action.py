#!/usr/bin/env python

import action

class SetSonarSeekTargetAction(action.Action):
	desired_target = ""


	def __init__(self, my_autonomy, target):
		self.action_name = "Set Sonar Seek Target"
		self.desired_target = target
		self.my_autonomy = my_autonomy


	def execute(self):
		self.print_start()
		self.my_autonomy.set_sonar_seek_target(self.desired_target)
		self.print_success()
		return True

	
