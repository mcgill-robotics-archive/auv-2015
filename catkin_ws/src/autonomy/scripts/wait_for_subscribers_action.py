#!/usr/bin/env python
"""@package docstring
"""

import action

class WaitForSubscribersAction(action.Action):
  subscriptions = []

  def __init__(self, my_autonomy):
    self.action_name = "Wait for Subscribers"
    self.my_autonomy = my_autonomy
    self.subscriptions = my_autonomy.get_subscribers()

  def execute(self):
    self.print_start()
#    while (len(self.subscriptions) > 0):
#      if(self.subscriptions[0].get_num_connections() > 0):
#        self.subscriptions.remove(0)
    self.print_success()
    return True