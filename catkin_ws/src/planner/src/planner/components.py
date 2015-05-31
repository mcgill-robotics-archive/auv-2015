from planner.primitives import SetVelocityState
from auv_msgs.msg import SetVelocity

class SetYawState(SetVelocityState):
    '''
    A state to set the yaw of the robot. Terminates when the yaw error
    is less than tolerance twice in a row.
    '''

    def __init__(self, yaw, tol=0.01):
        msg = SetVelocity()
        msg.yaw = yaw
        self.tol = tol
        super(SetYawState, self).__init__(msg, pre_execute_cb=self.init_state,
                feedback_cb=self.feedback_cb)

    def init_state(self, ud):
        self.in_tol = False

    def feedback_cb(self, feedback):
        in_tol = abs(feedback.yaw_error) < self.tol
        if in_tol and self.in_tol:
            self.exit_success()
        else:
            self.in_tol = in_tol
