#surface_action.py

import action


class SurfaceAction(action.Action):
	desired_velocity = 0

	def __init__(self, my_autonomy, velocity):
		self.action_name = "Surface Action"
		self.my_autonomy = my_autonomy
		self.desired_velocity = velocity



	def surface(self, velocity):
		desired_velocity = velocity
		self.my_autonomy.set_velocity(desired_velocity)


	def execute(self):
		self.print_start()
		
		#need to be above pinger
		while(self.my_autonomy.get_transform(self, "pinger") != (0,0)):
			if (self.my_autonomy.get_transform(self, "pinger")[0] < 1):
				break
		
		surface(self, desired_velocity)

		self.print_success()
		return True