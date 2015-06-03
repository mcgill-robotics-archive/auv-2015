import smach
from planner.primitives import SetVelocityState
from auv_msgs.msg import SetVelocity

def createMoveForwardState(speed, duration):
    '''
    Creates a state which moves the robot forward at the given speed for the
    given duration. duration should be a rospy.Duration instance.
    '''
    def goal_cb(user_data, goal):
        goal.cmd.yaw = user_data.yaw_setpoint
        goal.cmd.surgeSpeed = speed 
        goal.cmd.depth = user_data.depth_setpoint

    return SetVelocityState(goal_cb, duration=duration,
                input_keys=['yaw_setpoint', 'depth_setpoint'],
                output_keys=['yaw_setpoint', 'depth_setpoint'])

def createSetDepthState(depth, tolerance=0.01):
    '''
    Creates a state which moves the robot to the given depth. The depth must be
    within the tolerance twice in a row before the state exits.
    '''
    def goal_cb(user_data, goal):
        goal.cmd.yaw = user_data.yaw_setpoint
        goal.cmd.depth = depth
    
    def feedback_cb(state, feedback):
        if abs(feedback.depth_error) < tolerance:
            state.exit_success()

    def success_cb(user_data):
        user_data.depth_setpoint = depth

    return SetVelocityState(goal_cb, feedback_cb=feedback_cb,
            success_cb=success_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])

def createSetYawState(yaw_offset, tolerance=0.01):
    '''
    Creates a state to set the yaw of the robot. Terminates when the yaw error
    is less than tolerance twice in a row.
    '''
    def goal_cb(user_data, goal):
        goal.cmd.yaw = user_data.yaw_setpoint + yaw_offset
        goal.cmd.depth = user_data.depth_setpoint

    def feedback_cb(state, feedback):
        if abs(feedback.yaw_error) < tolerance:
            state.exit_success()

    def success_cb(user_data):
        user_data.yaw_setpoint += yaw_offset

    return SetVelocityState(goal_cb, feedback_cb=feedback_cb, 
            success_cb = success_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])


