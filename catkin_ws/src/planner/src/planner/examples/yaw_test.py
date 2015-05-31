#! /usr/bin/env python2.7
import smach
import smach_ros
import rospy
import math
from auv_msgs.msg import SetVelocity, SetVelocityGoal, SetVelocityAction
from threading import Thread
from planner.components import SetYawState
from smach_ros import SimpleActionState

rospy.init_node('yaw_test')
sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
cmd = SetVelocity()
cmd.yaw = 0.1
goal = SetVelocityGoal()
goal.cmd = cmd
sas = SimpleActionState('/controls', SetVelocityAction, goal=goal)
with sm:
#    smach.StateMachine.add('SAS',sas) 
    smach.StateMachine.add('Yaw 0', SetYawState(0),
            transitions={'succeeded':'Yaw pi/2'})
    smach.StateMachine.add('Yaw pi/2', SetYawState(math.pi/2),
            transitions={'succeeded':'Yaw 0','aborted':'aborted','preempted':'preempted'})

sis = smach_ros.IntrospectionServer('yaw_test', sm, '/SM_ROOT')
sis.start()

# In order to get smach to respond to Ctrl+c we run it in a different thread
# and request a preempt on ctrl+c.
smach_thread = Thread(target=sm.execute)
smach_thread.start()
# It is critical to use the on_shutdown method to request the preempt rather than
# waiting until after rospy spin to do so. Otherwise, the state machine will not respond
# to ctrl+c. I don't know why
rospy.on_shutdown(sm.request_preempt)
rospy.spin()
print 'Preempt requested'
smach_thread.join()
print 'thread joined'
sis.stop()
