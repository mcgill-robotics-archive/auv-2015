#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot in the y-axis at a given velocity
-positive is to the right relative to the robot
"""

import action

class YAxisMovementAction(action.Action):
  desired_velocity = 0

  def __init__(self, my_autonomy, velocity):
    self.action_name = "Y Axis Movement"
    self.desired_velocity = velocity
    self.my_autonomy = my_autonomy

  def execute(self):
    self.print_start()
    velocity_array = [0, self.desired_velocity, 0, 0, 0, 0]
    self.my_autonomy.set_velocity(velocity_array)
    self.print_success()
    return True