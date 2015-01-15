#!/usr/bin/env python
"""@package docstring
This module is responsible for generating the routine
as well as controlling the chaning of tasks
"""

import rospy
import gate_task

task_stack = []

def load_task_stack(my_autonomy):
	my_gate_task = gate_task.GateTask(my_autonomy)
	task_stack.insert(0, my_gate_task)

def run_task():
	while(len(task_stack) > 0):
		current_task = task_stack.pop()
		if (not current_task.run_task()):
			return False
		rospy.loginfo("Routine completed successfully")
	return True