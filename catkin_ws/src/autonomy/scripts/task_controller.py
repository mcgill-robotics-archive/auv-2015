#!/usr/bin/env python

import rospy
import task
import gate_task

task_directory = {'gate': None, 'lane': None, 'buoy': None, 'marker': None, 'torpedo': None, 'maneuver': None, 'hydrophones': None}
routine = {'gate': None, 'lane': None, 'buoy': None, 'marker': None, 'torpedo': None, 'maneuver': None, 'hydrophones': None}

#converts to corresponding task object
def populate_tasks():
	global task_directory
	task_directory['gate'] = 1
	task_directory['lane'] = 2
	task_directory['buoy'] = 3
	task_directory['marker'] = 4
	task_directory['torpedo'] = 5
	task_directory['maneuver'] = 6
	task_directory['hydrophones'] = 7

#defines order in which tasks are executed
def populate_routine():
	global routine
	routine['gate'] = 'lane'
	routine['lane'] = 'buoy'
	routine['buoy'] = 'marker'
	routine['marker'] = 'torpedo'
	routine['torpedo'] = 'maneuver'
	routine['maneuver'] = 'hydrophones'
	routine['hydrophones'] = ''

def next_task(next_task):
	global task_directory, routine
	print task_directory[routine[next_task]]	
###################
def temp():
	my_task = gate_task.Gate_Task(1)
	my_task.execute()	