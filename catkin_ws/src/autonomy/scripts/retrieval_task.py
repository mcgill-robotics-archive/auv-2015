#recoverytask.py

import task
import set_sonar_target
import set_cv_target
##namechange TBD
import forward_movement_action
import get_transforms

class recoveryTask(task.Task):

		def __init__(self, phase, my_autonomy):
			self.task_name = "Retrieval"
			self.phase = phase
			self.my_autonomy = my_autonomy
			self.load_action_stack()

		def __init__(self, my_autonomy):
			self.task_name = "Retrieval"
			self.phase = 1
			self.my_autonomy = my_autonomy
			self.load_action_stack()
	
		def load_action_stack(self):
			if (self.phase > 0):
				#need to add/create initial methods for grabbing/placing object
				my_path_detection = set_cv_target.SetBallAction(self.my_autonomy, pathColor)
				my_forward_movement_action = forward_movement_action.ForwardMovementAction(self.my_autonomy, 3)
				my_position = get_transforms.GetTransforms(self.my_autonomy, Object)
				retrieval_of_pinger = set_sonar_target.SetSonarTarget(self.my_autonomy, pinger)
				
				self.action_stack_insert(0, retrieval_of_pinger)
				self.action_stack_insert(0, move_to_pinger)
				self.action_stack_insert(0, surface_in_oct)
				self.action_stack_insert(0, retrieval_of_object)
				self.action_stack_insert(0, my_path_detection)
				self.action_stack_insert(0, my_forward_movement_action)
				self.action_stack_insert(0, surface_in_oct)
				self.action_stack_insert(0, placement_of_object)