#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot in the x-axis at a given velocity for a given time 
"""

import action

class XAxisMovementAction(action.Action):
  desired_velocity = 0

  def __init__(self, my_autonomy, velocity, duration):
    self.action_name = "X Axis Movement Timed"
    self.desired_velocity = velocity
    self.my_autonomy = my_autonomy

  def execute(self):
    self.print_start()
    while duration != 0:
      velocity_array = [self.desired_velocity, 0, 0, 0, 0, 0]
      self.my_autonomy.set_velocity(velocity_array)
      duration--
    self.print_success()
    return True