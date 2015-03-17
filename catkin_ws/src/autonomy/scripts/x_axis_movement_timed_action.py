#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot in the x-axis at a given velocity for a given time 
"""

import action

class XAxisMovementTimedAction(action.Action):
  desired_velocity = 0

  def __init__(self, my_autonomy, velocity, duration):
    self.action_name = "X Axis Movement Timed"
    self.desired_velocity = velocity
    self.my_autonomy = my_autonomy
    self.duration = duration

  def execute(self):
    self.print_start()
    while self.duration != 0:
      velocity_array = [self.desired_velocity, 0, 0, 0, 0, 0]
      self.my_autonomy.set_velocity(velocity_array)
      self.duration = self.duration -1
    self.print_success()
    return True
