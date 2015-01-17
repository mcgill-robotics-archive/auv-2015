#!/usr/bin/env python

import task
import wait_for_subscribers_action

class InitTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Initialization"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Initialization"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    if (self.phase > 0):
      my_wait_for_subscribers_action = wait_for_subscribers_action.WaitForSubscribersAction(self.my_autonomy)
      self.action_stack.insert(0, my_wait_for_subscribers_action)