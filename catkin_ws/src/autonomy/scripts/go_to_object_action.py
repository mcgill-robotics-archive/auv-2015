#!/usr/bin/env python
"""@package docstring
This module is a class that send the robot to a position from a cv message 
"""

import action
import math

class GoToObjectAction(action.Action):
  target_frame = None
  x_distance_away = 0
  y_distance_away = 0
  z_distance_away = 0
  roll_distance_away = 0
  pitch_distance_away = 0
  yaw_distance_away = 0

  def __init__(self, my_autonomy, target_frame, x_distance_away,
                 y_distance_away = 0, z_distance_away = 0, roll_distance_away = 0,
                 pitch_distance_away = 0, yaw_distance_away = 0):
    self.action_name = "Closed Loop Movement"
    self.my_autonomy = my_autonomy
    self.target_frame = target_frame
    self.x_distance_away = x_distance_away
    self.y_distance_away = y_distance_away
    self.z_distance_away = z_distance_away
    self.roll_distance_away = roll_distance_away
    self.pitch_distance_away = pitch_distance_away
    self.yaw_distance_away = yaw_distance_away

  def out_of_bound(self):
    POSITION_BOUND = 1
    ANGLE_BOUND = 1
    (position, angles) = self.my_autonomy.get_transform(self.target_frame)
    diff_x = position.x
    diff_y = position.y
    diff_z = position.z
    diff_roll = angles[0]
    diff_pitch = angles[1]
    diff_yaw = angles[2]

    if(math.abs(diff_x) < POSITION_BOUND and math.abs(diff_y) < POSITION_BOUND and 
       math.abs(diff_z) < POSITION_BOUND and math.abs(diff_roll) < ANGLE_BOUND and 
       math.abs(diff_pitch) < ANGLE_BOUND and math.abs(diff_yaw) < ANGLE_BOUND):
      return False
    else:
      return True

  def execute(self):
    self.print_start()
    while (self.out_of_bound()): 
      self.my_autonomy.set_position(self.x_distance_away, self.y_distance_away, 
          self.z_distance_away, self.roll_distance_away, self.pitch_distance_away,
          self.yaw_distance_away, self.target_frame)
    #stop command
    self.my_autonomy.set_velocity(0,0,0,0,0,0)
    self.print_success()
    return True
