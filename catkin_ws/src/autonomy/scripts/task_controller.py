#!/usr/bin/env python
"""@package docstring
This module is responsible for generating the routine
as well as controlling the chaning of tasks
"""

import rospy
import test_rig

import init_task
import gate_task
import path_task

class TaskController():
  my_autonomy = None
  task_stack = []
  test_val = False

  def __init__(self, my_autonomy):
    self.my_autonomy = my_autonomy
    self.load_task_stack()

  def load_task_stack(self):
    if(self.test_val):
      my_test_rig = test_rig.TestRig(self.my_autonomy)
      self.task_stack.insert(0, my_test_rig)
    else:
      my_init_task = init_task.InitTask(self.my_autonomy)
      self.task_stack.insert(0, my_init_task)
      my_gate_task = gate_task.GateTask(self.my_autonomy)
      self.task_stack.insert(0, my_gate_task)
      my_path_task = path_task.PathTask(self.my_autonomy)
      self.task_stack.insert(0, my_path_task)

  def run_routine(self):
    self.my_autonomy.print_info("-Routine: startinng")
    while(len(self.task_stack) > 0):
      current_task = self.task_stack.pop()
      if (not current_task.run_task()):
        return False
      self.my_autonomy.print_info("-Routine: success")
    return True
