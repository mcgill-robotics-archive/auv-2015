#!/usr/bin/env python

import task
import x_axis_movement_action
import y_axis_movement_action
import z_axis_movement_action
import roll_movement_action
import pitch_movement_action
import yaw_movement_action
import get_transform_action
import set_sonar_seek_target_action
import set_sonar_track_target_action
import set_cv_target_action

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