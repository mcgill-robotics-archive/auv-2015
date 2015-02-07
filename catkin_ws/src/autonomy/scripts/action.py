#!/usr/bin/env python
"""@package docstring
This module is an abstract class to allow the interchanging of different actions
"""

from abc import ABCMeta, abstractmethod

class Action():
  __metaclass__ = ABCMeta
  my_autonomy = None
  action_name = ""

  def __init__(self, my_autonomy):
    self.my_autonomy = my_autonomy

  def print_start(self):
    print_string = "---Action: " + self.action_name + " starting" 
    self.my_autonomy.print_info(print_string)

  def print_success(self):
    print_string = "---Action: " + self.action_name + " success" 
    self.my_autonomy.print_info(print_string)

  def print_failure(self, error_state = ""):
    print_string = "---Action: " + self.action_name + " FAILURE" + error_state 
    self.my_autonomy.print_info(print_string)   

  @abstractmethod
  def execute(self):  
    raise NotImplementedError()