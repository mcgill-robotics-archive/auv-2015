#!/usr/bin/env python

import task
import x_axis_movement_action

class GateTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Gate"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Gate"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    if (self.phase > 0):
      my_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 3)
      self.action_stack.insert(0, my_x_axis_movement_action)