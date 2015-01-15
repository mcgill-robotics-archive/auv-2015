#!/usr/bin/env python
"""@package docstring
This module is an abstract class to allow the interchanging of different tasks
"""

from abc import ABCMeta, abstractmethod

class Task():
    __metaclass__ = ABCMeta
    phase = -1
    my_autonomy = None
    action_stack = []

    def __init__(self, phase, my_autonomy):
        self.phase = phase
        self.my_autonomy = my_autonomy
    
    @abstractmethod
    def load_action_stack(self):
        raise NotImplementedError()

    def run_task(self):	
		while(len(self.action_stack) > 0):
			current_action = self.action_stack.pop()
			if (not current_action.execute()):
				return False
		self.my_autonomy.print_info("Task completed successfully")		
		return True