import action


class HydrophoneTargetAction(action.Action):


	def __init__(self, target, my_autonomy):
		self.action_name = "Hydrophone Target Action"
		target = None
		self.my_autonomy = my_autonomy

	def execute(self):
		self.print_start()
		if (self.my_autonomy.hydrophone_target_action(target)):
			self.print_success()
			return True
		else:
			self.print_failure()
			return False
