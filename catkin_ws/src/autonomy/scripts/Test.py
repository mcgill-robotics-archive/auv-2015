#!/usr/bin/env python
import rospy
#python abstract class thingies
from abc import ABCMeta, abstractmethod
from multiprocessing import Process, Value


a = 1
b = 2
c = 3
myArray = {'A': a, 'B': b, 'C': c}


class TestClass():
	__metaclass__ = ABCMeta
	phase = -1
	def __init__(self, newPhase):
		self.phase = newPhase
	def execute(self):
		print self.phase
	@abstractmethod	
	def InheritCheck(self):
		raise NotImplementedError()
	@abstractmethod	
	def InheritCheck2(self):
		raise NotImplementedError()

class ChildClass(TestClass):
	def __init__(self, newPhase):
		self.phase = newPhase
	def InheritCheck(self):
		print "it worked"
	def InheritCheck2(self):
		print "it worked again"

def thread1(n):
	while(True):
		print n.value

def thread2(n):
		n.value += 1

def my_globals():
	global a, b, c
	print myArray
	myArray['A'] += 1
	b = -b
	c *= c
	print myArray

if __name__ == '__main__':
#inheritance
#	y = ChildClass(2)
#	y.execute()
#	y.InheritCheck()
#	y.InheritCheck2()

#multithreading
#	num = Value('i', 0)
#	p = Process(target=thread2(num))
#	p.start()
#	p.join()
#	print 6
#	thread1(num)

# global avlue updating
	my_globals()
