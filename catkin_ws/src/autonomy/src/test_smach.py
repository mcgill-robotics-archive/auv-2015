#! /usr/bin/env python2.7
import smach
import smach_ros
import rospy

# State Foo
class Foo(smach.State):
    def __init__(self, outcomes=['outcome1', 'outcome2']):
        super(Foo, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        return 'outcome1'


# State Bar
class Bar(smach.State):
    def __init__(self, outcomes=['outcome3', 'outcome4']):
        super(Bar, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        return 'outcome4'


# State Bas
class Bas(smach.State):
    def __init__(self, outcomes=['outcome5']):
        super(Bas, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        return 'outcome5'

sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
with sm:
    smach.StateMachine.add('FOO', Foo(),
            transitions={'outcome1':'BAR',
                    'outcome2':'outcome4'})
    smach.StateMachine.add('BAR', Bar(),
            transitions={'outcome4':'FOO', 'outcome3':'outcome5'})

rospy.init_node('test_smach')
sis = smach_ros.IntrospectionServer('test_smach', sm, '/SM_ROOT')
sis.start()
sm.execute()
rospy.spin()
sis.stop()
