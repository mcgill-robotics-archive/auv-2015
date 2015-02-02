#!/usr/bin/env python

import task
import x_axis_movement_action
import set_cv_target_action
import go_to_object

class BallTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Buoy"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Buoy"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    if (self.phase > 0):
	  my_first_ball = set_cv_target_action.SetCVTargetAction(self.my_autonomy, firstBall)	
	  my_second_ball = set_cv_target_action.SetCVTargetAction(self.my_autonomy, secondBall)	
	  go_to_my_first_ball = go_to_object.GoToObject(self.my_autonomy,firstBall,3)
	  go_to_my_second_ball = go_to_object.GoToObject(self.my_autonomy,secondBall,3)
      	  my_forward_movement_action = x_movement_action.XMovementAction(self.my_autonomy, 3)
	  my_backward_movement_action = x_movement_action.XMovementAction(self.my_autonomy, -3)
      	  self.action_stack.insert(0, my_first_ball)
	  self.action_stack.insert(1, go_to_my_first_ball)
	  self.action_stack.insert(2, my_forward_movement_action)
	  self.action_stack.insert(3, my_backward_movement_action)
	  self.action_stack.insert(4, my_second_ball)
      	  self.action_stack.insert(5, go_to_my_second_ball)
	  self.action_stack.insert(6, my_forward_movement_action)
	  self.action_stack.insert(7, my_backward_movement_action)
