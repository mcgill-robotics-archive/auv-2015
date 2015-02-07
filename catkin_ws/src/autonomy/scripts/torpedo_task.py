#!/usr/bin/env python

import task
import set_cv_target_action
import go_to_object_action
import fire_torpedo_action
import x_axis_movement_action

class TorpedoTask(task.Task):
  def __init__(self, phase, my_autonomy):
    self.task_name = "Torpedo"
    self.phase = phase
    self.my_autonomy = my_autonomy
    self.load_action_stack()

  def __init__(self, my_autonomy):
    self.task_name = "Torpedo"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    #going to get cutout names from XML eventually, so put parsers here
    primary_cutout = "1776"
    secondary_cutout = "2001"
    if (self.phase > 0):
#      setCVtarget(torpedo_board)
      find_torpedo_board_action = set_cv_target_action.SetCVTargetAction(self.my_autonomy, "torpedo_board")
      self.action_stack.insert(0, find_torpedo_board_action)
#      open_loop til visible
      my_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 1)
      search_for_torpedo_board_action = open_loop_search_action.OpenLoopSearchAction(self.my_autonomy, "/torpedo_board", my_axis_movement_action)
#      closed_loop to move to position
      move_to_torpedo_board_action = go_to_object_action.GoToObjectAction(self.my_autonomy, "/torpedo_board", 2)
      self.action_stack.insert(0, move_to_torpedo_board_action)
#      setCVtarget(primary_cutout)
      find_primary_cutout_action = set_cv_target_action.SetCVTargetAction(self.my_autonomy, primary_cutout)
      self.action_stack.insert(0, find_primary_cutout_action)
#      closed_loop to move to firing position
      move_to_primary_firing_action = go_to_object_action.GoToObjectAction(self.my_autonomy, primary_cutout, 1)
      self.action_stack.insert(0, move_to_primary_firing_action)

#          ...remove cover routine...

#      fire torpedo!
      my_fire_left_torpedo_action = fire_torpedo_action.FireTorpedoAction(self.my_autonomy, "left")
      self.action_stack.insert(0, my_fire_left_torpedo_action)
#      closed_loop back to initial position
      self.action_stack.insert(0, move_to_torpedo_board_action)
#      setCVtarget(secondary_cutout)
      find_secondary_cutout_action = set_cv_target_action.SetCVTargetAction(self.my_autonomy, secondary_cutout)
      self.action_stack.insert(0, find_secondary_cutout_action)
#      closed_loop to move to firing position
      move_to_secondary_firing_action = go_to_object_action.GoToObjectAction(self.my_autonomy, secondary_cutout, 1)
      self.action_stack.insert(0, move_to_secondary_firing_action)
#      fire torpedo!
      my_fire_right_torpedo_action = fire_torpedo_action.FireTorpedoAction(self.my_autonomy, "right")
      self.action_stack.insert(0, my_fire_right_torpedo_action)
#      move up over torpedo_board
      move_over_torpedo_board_action = go_to_object_action.GoToObjectAction(self.my_autonomy, "/torpedo_board", 2, 0, 1)
      self.action_stack.insert(0, move_over_torpedo_board_action)
#      move forward
      my_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 1)
      self.action_stack.insert(0, my_x_axis_movement_action)
