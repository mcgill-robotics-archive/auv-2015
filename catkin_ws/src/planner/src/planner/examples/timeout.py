#! /usr/bin/env python2.7
import smach
import smach_ros
import rospy

# This is an example of two states running in parallel, one of which
# will preempt the other after a certain amount of time.

# State Foo
class Loop(smach.State):
    def __init__(self, outcomes=['preempted', 'try_again']):
        super(Loop, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        while not self.preempt_requested():
            pass
        return 'preempted'

csm = smach.Concurrence(['success', 'timer_failure', 'preempted'], 
        default_outcome='timer_failure', 
        # Preempt all children after first one finishes
        child_termination_cb = lambda x: True,
        outcome_map={'success':{'Timer':'false'},
                'timer_failure':{'Timer':'true'},
                'preempted':{'Timer':'preempted'}})
with csm:
#    smach.Concurrence.add('FOO', Foo())
    timer = smach_ros.ConditionState(lambda x: False,
            timeout=rospy.Duration(10.0), max_checks=-1)
    csm.add('Timer', timer)
    csm.add('Compliant Loop', Loop())


rospy.init_node('test_smach')
sis = smach_ros.IntrospectionServer('test_smach', csm, '/SM_ROOT')
sis.start()
csm.execute()
rospy.spin()
sis.stop()
