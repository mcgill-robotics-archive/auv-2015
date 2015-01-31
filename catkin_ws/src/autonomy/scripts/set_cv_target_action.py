#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot forward at a given velocity
"""

import action

class SetCVTargetAction(action.Action):
  desired_velocity = 0
  cv_target = None

  def __init__(self, my_autonomy, new_cv_target):
    self.action_name = "Set CV Target"
    self.my_autonomy = my_autonomy
    self.cv_target = new_cv_target

  def execute(self):
    self.print_start()
    self.my_autonomy.set_cv_target(self.cv_target)
    self.print_success()
    return True