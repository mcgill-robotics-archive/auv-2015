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

    def __init__(self, goal_cb, duration=None, feedback_cb=None,
            success_cb=None, input_keys=[], output_keys=[]):
        '''
        goal_cb -- Callback called when the state becomes active. 
            Variables which need to be initialized each time the state starts
            should be initialized here. Will be called with the user_data as an
            argument. It should return a SetVelocity message.
        duration -- How long to allow controls to work.
        feedback_cb -- Callback to recieve feedback from the action server.
        success_cb -- Called on success. Do cleanup here.
        '''
        self.goal_cb = goal_cb
        self.duration = duration
        self.feedback_cb = feedback_cb
        self.success_cb = success_cb
        super(SetVelocityState, self).__init__(
                'controls',
                SetVelocityAction, 
                goal_cb=self._goal_cb,
                result_cb=self.result_cb,
                input_keys=input_keys,
                output_keys=output_keys
        )

    def _goal_cb(self, user_data, goal):
        # Initialize this state
        self.requested_preempt = False
        self.initial_time = rospy.Time.now() 
        
        # Get the goal to send to controls
        self.goal_cb(user_data, goal)

    def _goal_feedback_cb(self, feedback):
        super(SetVelocityState, self)._goal_feedback_cb(feedback)

        # Exit if we timed out.
        if (self.duration is not None 
                and rospy.Time.now() - self.initial_time > self.duration):
            self.exit_success()
        elif self.feedback_cb is not None:
            self.feedback_cb(self, feedback)

    def result_cb(self, user_data, goal_status, goal_result):
        # If we preempted based on the timeout, the result should be success
        if self.requested_preempt and goal_status == GoalStatus.PREEMPTED:
            if self.success_cb is not None:
                self.success_cb(user_data)
            return 'succeeded'
        # For all other cases, we let the super handle it, so no need to 
        # return anything.

    def exit_success(self):
        # This will cancel the pending action and transition 
        # to the next state. The preempt is redefined to be success in result_cb
        self.requested_preempt = True
        self.request_preempt()
        
