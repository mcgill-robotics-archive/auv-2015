import task
import set_sonar_target
import set_cv_target
import x_axis_movement_action
import get_transforms
import go_to_object_action

class recoveryTask(task.Task):

		def __init__(self, phase, my_autonomy):
			self.task_name = "Retrieval Task"
			self.phase = phase
			self.my_autonomy = my_autonomy
			self.load_action_stack()

		def __init__(self, my_autonomy):
			self.task_name = "Retrieval Task"
			self.phase = 1
			self.my_autonomy = my_autonomy
			self.load_action_stack()
	
		def load_action_stack(self):
			#soon to be parsed from XML
			pathColor = "orange"
			targetObject = "pole"

			if (self.phase > 0):

				#Use hydrophones to detect pinger
				retrieval_of_pinger_action = hydrophone_target_action.HydrophoneTargetAction(self, self.my_autonomy, "pinger")
				self.action_stack_insert(0, retrieval_of_pinger_action)

				#GoToObject(pinger)
				move_to_pinger_action = go_to_object_action.GoToObjectAction(self.my_autonomy, "pinger", 2)
				self.action_stack_insert(0, move_to_pinger_action)

				#SURFACE WHEN ABOVE PINGER
				surface_action = surface_action.SurfaceAction(self.my_autonomy, [0,0,3,0,0,0])
				self.action_stack_insert(0, surface_action)
				
				#DESCEND (don't need to check again)
				descend_action = z_axis_movement_action.ZAxisMovementAction(self.my_autonomy, [0,0,-3,0,0,0])
				self.action_stack_insert(0, descend_action)

				#setSonarTarget(pole)
				retrieval_of_object_action = set_sonar_seek_target_action.SetSonarSeekTargetAction(self.my_autonomy, targetObject)
				self.action_stack_insert(0, retrieval_of_object_action)

				#grab first object
				grab_object_action = grab_object_action.GrabObjectAction(self.my_autonomy, "pole")
				self.action_stack_insert(0, placement_of_object_action)
				
				#setCVTarget(pathmarker) to go to next octagon
				my_path_detection_action = set_cv_target.SetCVTargetAction(self.my_autonomy, pathColor)
				self.action_stack_insert(0, my_path_detection_action)
				
				#go to next octagon
				go_towards_second_octagon_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 3)
				self.action_stack_insert(0, go_towards_second_octagon_action)
				
				#surface above second pinger
				self.action_stack_insert(0, surface_action)

				#descend again
				self.action_stack_insert(0, descend_action)

				#place first object on pole
				placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(self.my_autonomy, "pole")
				self.action_stack_insert(0, placement_of_object_action)

				#grab second object
				placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(self.my_autonomy)
				self.action_stack_insert(0, placement_of_object_action)

				#detect original pinger
				retrieval_of_pinger_action = set_sonar_seek_target_action.SetSonarSeekTargetAction(self, self.my_autonomy, pinger)
				self.action_stack_insert(0, retrieval_of_pinger_action)

				#go back to original octagon
				move_to_pinger_action = go_to_object_action.GoToObjectAction(self.my_autonomy, "pinger", 2)
				self.action_stack_insert(0, move_to_pinger_action)

				#place object again
				placement_of_object_action = placement_of_object_action.PlacementOfObjectAction(self.my_autonomy)
				self.action_stack_insert(0, placement_of_object_action)

				#surface in original octagon
				surface_action = z_axis_movement_action.ZAxisMovementAction(self.my_autonomy, [0,0,3,0,0,0])
				self.action_stack_insert(0, surface_action)