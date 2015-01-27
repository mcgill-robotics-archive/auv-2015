#!/usr/bin/env python

import action

class GetTransforms(action.Action):
	 search_object = ""

	def __init__(self, my_autonomy, Object):
		self.action_name = "Get Tranforms"
		self.search_object = Object
		self.my_autonomy = my_autonomy

	def execute(self):
		self.print_start()
			target_frame = Object
		self.my_autonomy.get_transform(target_frame)
		self.print_success()
		return True
