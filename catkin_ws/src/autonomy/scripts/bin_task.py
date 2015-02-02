#!/usr/bin/env python

import task
import x_axis_movement_action
import set_cv_target_action
import go_to_object

class BinTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Bins"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Bins"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    if (self.phase > 0):
      my_first_bin = set_cv_target_action.SetCvTarget(self.my_autonomy,firstBin)
      my_second_bin = set_cv_target_action.SetCvTarget(self.my_autonomy,secondBin)
      go_to_first_bin = go_to_object.GoToObject(self.my_autonomy,firstBin,0);
      go_to_second_bin = go_to_object.GoToObject(self.my_autonomy,secondBin,0);
      #drop the alien action should go here 
      self.action_stack.insert(0, my_first_bin)
      self.action_stack.insert(0, go_to_first_bin)
      #alien drop action should be added to stack here
      self.action_stack.insert(0, my_second_bin)
      self.action_stack.insert(0, go_to_second_bin)
      #drop second alien goes here