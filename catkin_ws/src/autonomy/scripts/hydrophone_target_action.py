import action


class HydrophoneTargetAction(action.Action):
		target = None

	def __init__(self, my_autonomy, target):
		self.action_name = "Hydrophone Target Action"
		self.target = target
		self.my_autonomy = my_autonomy

	def execute(self):
		self.print_start()
		if (self.my_autonomy.hydrophone_target_action(target)):
			self.print_success()
			return True
		else:
			self.print_failure()
			return False
