#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot forward at a given velocity
"""

import action

class SetBallAction(action.Action):
  desired_velocity = 0

  def __init__(self, my_autonomy, ballColor):
    self.action_name = ballColor
    self.my_autonomy = my_autonomy

  def execute(self):
    self.print_start()
	target_msg = ballColor
    self.my_autonomy.set_cv_target(target_msg)
    self.print_success()
    return True