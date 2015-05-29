from smach_ros import SimpleActionState
from auv_msgs.msg import SetVelocityAction

class SetVelocityState(SimpleActionState):
    '''
    A state to send open loop velocity commands to controls. Sends a
    SetVelocity command and exits after a given duration has passed,
    after which the SetVelocityAction is cancelled. To be used as a
    building block for more user friendly states.
    '''

    def __init__(self, velocity_cmd, duration, feedback_cb=None):
        '''
        velocity_cmd -- Message to send to controls
        duration -- How long to allow controls to work
        feedback_cb -- Callback to recieve feedback from the action server.
            Default: None
        '''
        super(SetVelocity, self).__init__(
                'controls', SetVelocityAction, goal=velocity_cmd)

    def _goal_feedback_cb(self, feedback):
        super(SetVelocityState, self)._goal_feedback_cb(feedback)
        print feedback
