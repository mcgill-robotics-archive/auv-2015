#!/usr/bin/env python

import action

class GetTransformAction(action.Action):
  target_frame = ""

  def __init__(self, my_autonomy, new_target_frame):
    self.action_name = "Get Tranform"
    self.my_autonomy = my_autonomy
    self.target_frame = new_target_frame

  def execute(self):
    self.print_start()
    target_frame = Object
#what is this doing? this method return something    
    self.my_autonomy.get_transform(target_frame)
		
    self.print_success()
    return True
