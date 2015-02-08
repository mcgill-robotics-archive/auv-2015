#!/usr/bin/env python

import task
import x_axis_movement_action
import set_cv_target_action
import go_to_object_action
import yaw_movement_action
import y_axis_movement_action

class ManeuverTask(task.Task):
	def __init__ (self, phase, my_autonomy):
		self.task_name = "Maneuver"
		self.phase = phase
		self.my_autonomy = my_autonomy
		self.load_action_stack()

	def __init__(self, my_autonomy):
		self.task_name = "Maneuver"
		self.phase = 1
		self.my_autonomy = my_autonomy
		self.load_action_stack()

	def load_action_stack(self):
		poleColour = "green"
		obstacleColour = "red"
			
		if (self.phase > 0):
			my_forward_x_axis_movment_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 3)

			my_pole_colour = set_cv_target_action.SetCVTargetAction(self.my_autonomy, poleColour)
			self.action_stack.insert(0, my_pole_colour)
			
			search_for_poles_action = open_loop_search_action.OpenLoopSearchAction(self.my_autonomy, my_x_axis_movement_action)
			go_to_poles = go_to_object_action.GoToObjectAction(self.my_autonomy, poleColour, 2)
			self.action_stack.insert(0, go_to_poles)
			
			my_obstacle_colour = set_cv_target_action.SetCVTargetAction(self.my_autonomy, obstacleColour)
			self.action_stack.insert(0, my_obstacle_colour)
			
			go_to_obstacle = go_to_object_action.GoToObject(self.my_autonomy, obstacleColour, 2)
			self.action_stack.insert(0, go_to_obstacle)
			
			rotate_to_sideways = yaw_movement_action.YawMovementAction(self.my_autonomy, 1) #how to stop at 90 degrees
			rotate_to_backwards = yaw_movement_action.YawMovementAction(self.my_autonomy, 1) #how to stop at 180 degrees
			self.action_stack.insert(0, rotate_to_sideways)
#			self.action_stack.insert(0, rotate_to_backwards)
			
			my_left_y_axis_movement_action = y_axis_movement_action.YAxisMovementAction(self.my_autonomy, -3)
			self.action_stack.insert(0, my_left_y_axis_movement_action)
#			my_backward_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, -3)
#			self.action_stack.insert(0, my_backward_x_axis_movement_action)
