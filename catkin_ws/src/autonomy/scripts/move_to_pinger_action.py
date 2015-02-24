#move_to_pinger_action.py

import action
import set_sonar_target


class MoveToPingerAction(action.Action):
	desired_velocity = 0


	def __init__(self, velocity):
		self.action_name = "Move to Pinger"
		self.desired_velocity = velocity
		self.my_autonomy = my_autonomy

	def execute(self):
		
		self.print_start()
		
		
		velocity_array = [self.desired_velocity, 0,0,0,0,0]
		self.my_autonomy.set_velocity(velocity_array)
		
		self.print_success()
		return True