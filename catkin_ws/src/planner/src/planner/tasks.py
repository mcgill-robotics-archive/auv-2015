import smach
from std_msgs.msg import Bool
import rospy
import smach_ros
import components
from auv_msgs.msg import SetCVTarget, SetCVTargetAction, SetCVTargetGoal
from primitives import InitializationState, SetVelocityState


def initialize(pause, countdown, depth):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['yaw_setpoint', 'depth_setpoint'])
    with sm:
        # Wait for mission switch
        smach.StateMachine.add(
            'Mission Switch',
            smach_ros.MonitorState('/mission', Bool, lambda x, y: not y.data),
            # This state should return invalid once mission is true
            transitions={'valid': 'aborted', 'invalid': 'Pause'})

        # Pause
        poll_rate = 0.1
        smach.StateMachine.add(
            'Pause',
            smach_ros.ConditionState(lambda x: False,
                                     poll_rate=rospy.Duration(0.1),
                                     max_checks=pause/poll_rate),
            # This state returns false after the period times out
            transitions={'true': 'aborted', 'false': 'Initialization'})

        # Horizon initialization.
        smach.StateMachine.add(
            'Initialization',
            InitializationState(countdown),
            transitions={'succeeded': 'Submerge'})

        # Submerge
        smach.StateMachine.add(
            'Submerge',
            SetVelocityState.create_set_depth_state(depth, tolerance=0.1),
            transitions={'succeeded': 'Yaw 0'})
    return sm


def lane_task():
    # Returns a state machine which will dead reckon towards the lane and
    # align over it using visual servoing.
    motion_sm = smach.StateMachine()
    with motion_sm:
        motion_sm.add_auto(
            'Switch Entry Point',
            components.switch_entry_point(motion_sm, 'Search Pattern'),
            connector_outcomes=['success'])
        motion_sm.add_auto(
            'Dead Reckon', components.dead_reckon_with_monitor(),
            connector_outcomes=['success'])
        motion_sm.add_auto(
            'Search Pattern', components.search_pattern_with_condition(),
            connector_outcomes=['success'])
        motion_sm.add('Visual Servo', components.visual_servo())
    tracking_concurrence = smach.Concurrence(
        ['success', 'lane_not_found', 'failed'],
        child_termination_cb=lambda x: True,
        default_outcome='failed',
        outcome_map={  # TODO:
            })
    with tracking_concurrence:
        tracking_concurrence.add('Motion', motion_sm)
        tracking_concurrence.add(
            'Tracking',
            smach_ros.SimpleActionState(
                'tracking',
                SetCVTargetAction,
                goal=SetCVTargetGoal(SetCVTarget.LANE)))
    retry_tracking_sm = smach.Iterator()
    with retry_tracking_sm:
        # On failure we retry. This will cause the robot to execute a search
        # pattern.
        retry_tracking_sm.set_contained_state(
            'Concurrence', tracking_concurrence, loop_outcomes=['failed'])
    sm = smach.Concurrence(
        ['success', 'lane_not_found', 'failed'],
        child_termination_cb=lambda x: True,
        default_outcome='failed',
        outcome_map={'success': {'Motion': 'success'},
                     'lane_not_found': {'Motion': 'lane_not_found'}})
    with sm:
        sm.add('Motion', retry_tracking_sm)
        sm.add('CV', smach_ros.SimpleActionState(
            'cv_server',
            SetCVTargetAction,
            goal=SetCVTargetGoal(SetCVTarget.LANE)))
    return sm
