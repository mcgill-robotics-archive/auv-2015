#!/usr/bin/env python

import task
import forward_movement_action

class BallTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Ball"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Ball"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    if (self.phase > 0):
	  my_first_ball = set_ball.SetBallAction(self.my_autonomy, firstBall)	
	  my_second_ball = set_ball.SetBallAction(self.my_autonomy, secondBall)	
      my_forward_movement_action = forward_movement_action.ForwardMovementAction(self.my_autonomy, 3)
	  my_backward_movement_action = forward_movement_action.ForwardMovementAction(self.my_autonomy, -3)
      self.action_stack.insert(0, my_first_ball)
	  self.action_stack.insert(1, my_forward_movement_action)
	  self.action_stack.insert(2, my_backward_movement_action)
	  self.action_stack.insert(3, my_second_ball)
	  self.action_stack.insert(4, my_forward_movement_action)
	  self.action_stack.insert(5, my_backward_movement_action)