#!/usr/bin/env python

import action
import set_cv_target_action
import go_to_object_action

class HailMarry(action.Action):

  def __init__(self, my_autonomy, cv_target):
    self.action_name = "Hail Marry"
    self.my_autonomy = my_autonomy
    self.cv_target = cv_target

  def execute(self):
    self.print_start()
    self.set_cv_target_action.SetCVTarget(self.my_autonomy, cv_target)
    self.go_to_object_action.GoToObject(self.my_autonomy,cv_target,3)
    self.my_autonomy.drop_marker("left")
    self.my_autonomy.drop_marker("right")
    self.print_success()
    return state
