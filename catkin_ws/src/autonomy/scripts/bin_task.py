#!/usr/bin/env python

import task
import x_axis_movement_action
import set_cv_target_action
import go_to_object_action
import set_claw_state_action
import hail_marry_action

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
      my_first_bin = set_cv_target_action.SetCVTargetAction(self.my_autonomy,"firstBin")
      my_second_bin = set_cv_target_action.SetCVTargetAction(self.my_autonomy,"secondBin")
      go_to_first_bin = go_to_object_action.GoToObjectAction(self.my_autonomy,"firstBin",0)
      go_to_second_bin = go_to_object_action.GoToObjectAction(self.my_autonomy,"secondBin",0)
      drop_left_bin = set_claw_state_action.SetClawStateAction(self.my_autonomy,"left")
      drop_right_bin = set_claw_state_action.SetClawStateAction(self.my_autonomy,"right")
      hail_marry_bin = hail_marry_action.HailMarry(self.my_autonomy,"lastBin")
      self.action_stack.insert(0, my_first_bin)
      self.action_stack.insert(0, go_to_first_bin)
      self.action_stack.insert(0, drop_left_bin)
      self.action_stack.insert(0, my_second_bin)
      self.action_stack.insert(0, go_to_second_bin)
      self.action_stack.insert(0, drop_right_bin)
      self.action_stack.insert(0, hail_marry_bin)
      testStack = self.get_stack()
    return testStack

