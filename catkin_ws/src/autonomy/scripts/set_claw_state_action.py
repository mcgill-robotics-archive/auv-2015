#!/usr/bin/env python

import action

class SetClawStateAction(action.Action):

  def __init__(self, my_autonomy, state):
    self.action_name = "Set Claw State"
    self.my_autonomy = my_autonomy
    self.state = state

  def execute(self):
    self.print_start()
    self.my_autonomy.drop_marker(state)
    self.print_success()
    return state
