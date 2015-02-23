#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot forward at a given velocity
"""

import action

class ForwardMovementAction(action.Action):
  desired_velocity = 0

  def __init__(self, my_autonomy, velocity):
    self.action_name = "Forward Movement"
    self.desired_velocity = velocity
    self.my_autonomy = my_autonomy

  def execute(self):
    self.print_start()
    velocity_array = [self.desired_velocity, 0, 0, 0, 0, 0]
    self.my_autonomy.set_velocity(velocity_array)
    self.print_success()
    return True
