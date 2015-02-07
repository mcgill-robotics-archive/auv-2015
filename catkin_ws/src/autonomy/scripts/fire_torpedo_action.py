#!/usr/bin/env python
"""@package docstring
This module is a class that fires a torpedo
"""

import action

class FireTorpedoAction(action.Action):
  torpedo_side = None

  def __init__(self, my_autonomy, side):
    self.action_name = "Torpedo"
    self.torpedo_side = side
    self.my_autonomy = my_autonomy

  def execute(self):
    self.print_start()
    if(self.my_autonomy.fire_torpedo(self.torpedo_side)):
      self.print_success()
      return True
    else:
      self.print_failure()
      return False