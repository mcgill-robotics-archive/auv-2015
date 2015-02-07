#!/usr/bin/env python
"""@package docstring
This module is a class that runs open loop control until the robot finds the
object it is looking for
"""

import action

class OpenLoopSearchAction(action.Action):
  target_frame = ""
  movement_action = None

  def __init__(self, my_autonomy, target_frame, basic_movement_action):
    self.action_name = "Open Loop Search"
    self.my_autonomy = my_autonomy
    self.target_frame = target_frame
    self.movement_action = basic_movement_action

  def execute(self):
    while(not self.my_autonomy.check_transform(self.target_frame)):
      self.movement_action.execute()
    return True  