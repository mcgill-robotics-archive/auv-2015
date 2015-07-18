import smach
import smach_ros
from primitives import SetVelocityState
import rospy
import tf
from math import pi
from auv_msgs.msg import SetPositionAction, CVTarget


def switch_entry_point(state_machine, label):
    # Returns a state which switches state_machine's entry point to the
    # specified label.
    state = smach.State(['succeeded'])

    def execute(user_data):
        state_machine.set_initial_state(label)
        return 'succeeded'
    state.execute = execute
    return state


def search_pattern():
    # For now just come to a stop and look for 15 seconds.
    # TODO: Do fancier search pattern
    return SetVelocityState.create_move_forward_state(0, rospy.Duration(15))


def search_pattern_with_condition():
    csm = smach.Concurrence(
        ['lane_tracked', 'lane_not_found'],
        child_termination_cb=lambda x: True,
        default_outcome='lane_not_found',
        outcome_map={'lane_tracked': {'Condition': 'false'}})
    with csm:
        def test_transform(frame1, frame2, timeout):
            # Tests whether the transform is available
            try:
                tf.waitForTransform(frame1, frame2, timeout)
                return True
            except tf.Exception:
                return False

        poll_rate = rospy.Duration(0.1)
        csm.add('Condition', smach_ros.ConditionState(
            lambda x: not test_transform('horizon', 'lane', 0.9 * poll_rate),
            poll_rate=poll_rate,
            max_checks=-1))
        csm.add('Search Pattern', search_pattern())
    return csm


def visual_servo():
    # Position ourselves over and align with the lane.
    def goal_cb(user_data, goal):
        # This transform will exist, since we only start visual servoing
        # once it does
        (trans, rot) = \
            tf.TransformListener().lookupTransform('horizon', 'lane')
        yaw = rot[2]
        # We attempt to align in the direction closest to our
        # starting direction
        if abs(yaw) > pi/2:
            goal.cmd.yaw = pi
            yaw += pi
        # Store the yaw, so that in case the visual servoing fails we will
        # at least be pointing in the right direction.
        user_data.yaw_setpoint += yaw

    return smach_ros.SimpleActionState(
        'controls_position',
        SetPositionAction,
        goal_cb=goal_cb,
        input_keys=['yaw_setpoint'],
        output_keys=['yaw_setpoint'])


def dead_reckon_with_monitor(yaw, speed, duration):
    csm = smach.Concurrence(
        ['lane_seen', 'lane_not_found', 'preempted', 'aborted'],
        child_termination_cb=lambda x: True,
        default_outcome='lane_not_found',
        outcome_map={'lane_seen': {'Monitor CV': 'invalid'}})
    with csm:
        csm.add(
            'Move Forward',
            SetVelocityState.create_move_forward_state(
                speed,
                rospy.Duration(duration)))
        csm.add(
            'Monitor CV',
            smach_ros.MonitorState(
                '/cv/identified_targets',
                CVTarget,
                lambda x, y: False))
    sm = smach.StateMachine(['lane_seen', 'lane_not_found',
                             'preempted', 'aborted'])
    with sm:
        sm.add(
            'Set Yaw',
            SetVelocityState.create_set_yaw_state(yaw),
            transitions={'succeeded': 'Concurrence'})
        sm.add('Concurrence', csm)
    return sm
