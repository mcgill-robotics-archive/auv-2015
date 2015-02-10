#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot to a position from a cv message 
"""

import action

class GoToObject(action.Action):
  desired_velocity = 0
  cv_target = None

  def __init__(self, my_autonomy, new_cv_target, distance_away):
    self.action_name = "Set Position from CV"
    self.my_autonomy = my_autonomy
    self.cv_target = new_cv_target

  def execute(self):
    self.print_start()
    while self.my_autonomy.get_transform(self,self.my_autonomy.set_position(0,0,0,0,0,0,self.cv_target)) != distance_away: """" While we are still some distance away fro target """
    	self.my_autonomy.set_position(0,0,0,0,0,0,self.cv_target)
	if self.my_autonomy.get_transform(self,self.my_autonomy.set_position(0,0,0,0,0,0,self.cv_target)) == distance_away:
		self.my_autonomy.set_velocity(0,0,0,0,0,0)
		break
    self.print_success()
    return True
