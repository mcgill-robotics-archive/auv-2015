#placement_of_object_action.py

import action


class PlacementOfObjectAction(action.Action):

	def __init__(self, my_autonomy, target):
		self.action_name = "Placement of Object"
		self.target = target
		self.my_autonomy = my_autonomy



	def execute(self):
		self.print_start()
		while (!self.my_autonomy.set_sonar_seek_target(self, target)):
			self.my_autonomy.set_sonar_seek_target(self, target)

		self.my_autonomy.toggle_grabber("front", "open")

		self.print_success()
		return True
		