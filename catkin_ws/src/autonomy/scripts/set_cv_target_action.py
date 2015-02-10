#!/usr/bin/env python
"""@package docstring
This module is a class that sends mesage for cv to look for  
"""

import action
import yaw_movement_action

class SetCVTargetAction(action.Action):
  desired_velocity = 0
  cv_target = None

  def __init__(self, my_autonomy, new_cv_target):
    self.action_name = "Set CV Target"
    self.my_autonomy = my_autonomy
    self.cv_target = new_cv_target

  def execute(self):
    self.print_start()
    while self.my_autonomy.target_was_found != True:
      self.my_autonomy.set_cv_target(self.cv_target)
      self.yaw_movement_action.YawMovementAction(self.my_autonomy, 3)
    self.print_success()
    return True
