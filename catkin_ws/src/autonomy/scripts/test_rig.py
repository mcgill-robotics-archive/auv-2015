#!/usr/bin/env python

import task
import x_axis_movement_action
import y_axis_movement_action
import z_axis_movement_action
import roll_movement_action
import pitch_movement_action
import yaw_movement_action
import set_sonar_seek_target_action
import set_sonar_track_target_action
import set_cv_target_action
import fire_torpedo_action
import open_loop_search_action
import go_to_object_action
import placement_of_object_action

class TestRig(task.Task):
  def __init__(self, my_autonomy):
    self.task_name = "Testing Rig"
    self.phase = 1
    self.my_autonomy = my_autonomy
    self.load_action_stack()    

  def load_action_stack(self):
    my_x_axis_movement_action = x_axis_movement_action.XAxisMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_x_axis_movement_action)

    my_y_axis_movement_action = y_axis_movement_action.YAxisMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_y_axis_movement_action)

    my_z_axis_movement_action = z_axis_movement_action.ZAxisMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_z_axis_movement_action)

    my_roll_movement_action = roll_movement_action.RollMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_roll_movement_action)

    my_pitch_movement_action = pitch_movement_action.PitchMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_pitch_movement_action)

    my_yaw_movement_action = yaw_movement_action.YawMovementAction(self.my_autonomy, 3)
    self.action_stack.insert(0, my_yaw_movement_action)

    my_set_sonar_seek_target_action = set_sonar_seek_target_action.SetSonarSeekTargetAction(self.my_autonomy, "string")
    self.action_stack.insert(0, my_set_sonar_seek_target_action)

    my_set_sonar_track_target_action = set_sonar_track_target_action.SetSonarTrackTargetAction(self.my_autonomy, "string")
    self.action_stack.insert(0, my_set_sonar_track_target_action)

    my_set_cv_target_action = set_cv_target_action.SetCVTargetAction(self.my_autonomy, "string")
    self.action_stack.insert(0, my_set_cv_target_action)

    my_fire_torpedo_action = fire_torpedo_action.FireTorpedoAction(self.my_autonomy, "left")
    self.action_stack.insert(0, my_fire_torpedo_action)

    my_go_to_object_action = go_to_object_action.GoToObjectAction(self.my_autonomy, "string", 1, 1, 1, 1, 1, 1)
#    self.action_stack.insert(0, my_go_to_object_action)

    my_open_loop_search_action = open_loop_search_action.OpenLoopSearchAction(self.my_autonomy, "board", my_y_axis_movement_action)
#    self.action_stack.insert(0, my_open_loop_search_action)

    my_placement_of_object_action = placement_of_object_action(self.my_autonomy, "pole")
    self.action_stack_insert(0, my_placement_of_object_action)

    my_surface_action = surface_action.SurfaceAction(self.my_autonomy, [0,0,0,0,0,0])
    self.action_stack_insert(0, my_surface_action)

    my_hydrophone_target_action = hydrophone_target_action.HydrophoneTargetAction(self.my_autonomy, "pinger")
    self.action_stack_insert(0, hydrophone_target_action)

    my_grab_object_action = grab_object_action.GrabObjectAction(self.my_autonomy, "object")
    self.action_stack_insert(0, my_grab_object_action)