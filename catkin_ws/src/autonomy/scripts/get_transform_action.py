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
#what is this doing? this method return something    
    transform = self.my_autonomy.get_transform(self.target_frame)
    position = transform[0]
    quaternion = transform[1]
    self.my_autonomy.set_position(position)
		
    self.print_success()
    return True
