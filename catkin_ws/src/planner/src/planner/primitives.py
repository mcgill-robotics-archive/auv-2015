from smach_ros import SimpleActionState
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal
from actionlib.simple_action_client import GoalStatus
import rospy

class SetVelocityState(SimpleActionState):
    '''
    A state to send open loop velocity commands to controls. Sends a
    SetVelocity command and exits after a given duration has passed,
    after which the SetVelocityAction is cancelled. To be used as a
    building block for more user friendly states.
    '''

    def __init__(self, velocity_cmd, duration=None,
            pre_execute_cb = None, feedback_cb=None):
        '''
        velocity_cmd -- Message to send to controls.
        duration -- How long to allow controls to work.
        pre_execute_cb -- Callback called when the state becomes active. 
            Variables which need to be initialized each time the state starts
            should be initialized here.
        feedback_cb -- Callback to recieve feedback from the action server.
        '''
        goal = SetVelocityGoal()
        goal.cmd = velocity_cmd
        self.duration = duration
        self.feedback_cb = feedback_cb
        self.pre_execute_cb = pre_execute_cb
        super(SetVelocityState, self).__init__(
                'controls',
                SetVelocityAction, 
                goal=goal,
                result_cb=self.result_cb
        )

    def execute(self, ud):
        print 'executing'
        self.requested_preempt = False
        self.initial_time = rospy.Time.now() 
        if self.pre_execute_cb:
            self.pre_execute_cb(ud)
        return super(SetVelocityState, self).execute(ud)

    def _goal_feedback_cb(self, feedback):
        super(SetVelocityState, self)._goal_feedback_cb(feedback)
        print feedback
        # Exit if we timed out.
        if (self.duration is not None 
                and rospy.Time.now() - self.initial_time > self.duration):
            self.exit_success()
        else:
            self.feedback_cb(feedback)

    def result_cb(self, user_data, goal_status, goal_result):
        # If we preempted based on the timeout, the result should be success
        if self.requested_preempt and goal_status == GoalStatus.PREEMPTED:
            return 'succeeded'
        # For all other cases, we let the super handle it, so no need to 
        # return anything.

    def exit_success(self):
        # This will cancel the pending action and transition 
        # to the next state. The preempt is redefined to be success in result_cb
        self.requested_preempt = True
        self.request_preempt()
        
