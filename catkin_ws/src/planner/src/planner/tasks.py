import smach
from std_msgs.msg import Bool
import rospy
import smach_ros
import components
from auv_msgs.msg import CVTarget, SetCVTargetAction, SetCVTargetGoal
from primitives import InitializationState, SetVelocityState, SetUserDataState


def mission_switch_wrapper(state_machine):
    # This will return a state macine containing @arg state_machine which
    # listens to mission switch to start and stop @arg state_machine.
    # @arg state_machine must have one outcome called 'succeeded', and may
    # also have outcomes called 'aborted' or 'preempted'.
    csm = smach.Concurrence(
        ['succeeded', 'aborted', 'preempted'],
        child_termination_cb=lambda x: True,
        default_outcome='aborted',
        outcome_map={
            'succeeded': {'Mission': 'succeeded'},
            'preempted': {'Monitor Mission': 'preempted'}})
    with csm:
        csm.add('Mission', state_machine)

        global prev_boolean
        prev_boolean = False

        def or_filter(boolean):
            global prev_boolean
            ret_val = boolean or prev_boolean
            prev_boolean = boolean
            return ret_val

        csm.add(
            'Monitor Mission',
            smach_ros.MonitorState(
                'mission',
                Bool,
                lambda x, y: or_filter(y.data)))

    sm = smach.StateMachine(['preempted', 'aborted'])
    with sm:
        # Wait for mission switch
        smach.StateMachine.add_auto(
            'Mission Switch',
            smach_ros.MonitorState('/mission', Bool, lambda x, y: not y.data),
            # This state should return invalid once mission is true
            connector_outcomes=['invalid'],
            transitions={'valid': 'aborted'})
        smach.StateMachine.add(
            'Mission with Monitor',
            csm,
            transitions={
                'aborted': 'Mission Switch',
                'succeeded': 'Mission Switch',
                # Yes we do want preempted to go back to mission switch. Some
                # of the states can return preempted witout ctrl+c.
                'preempted': 'Mission Switch'})
    return sm


def initialize(pause, countdown, depth):
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        output_keys=['yaw_setpoint', 'depth_setpoint'])
    with sm:
        # Create user data
        ud = smach.UserData()
        ud['yaw_setpoint'] = 0.0
        ud['depth_setpoint'] = 0.0
        smach.StateMachine.add_auto(
            'Set User Data',
            SetUserDataState(ud),
            connector_outcomes=['succeeded'])

        # Pause
        smach.StateMachine.add_auto(
            'Pause',
            smach_ros.ConditionState(lambda x: False,
                                     timeout=rospy.Duration(pause),
                                     max_checks=-1),
            # This state returns false after the period times out
            connector_outcomes=['false'],
            transitions={'true': 'aborted'})

        # Horizon initialization.
        smach.StateMachine.add_auto(
            'Initialization',
            InitializationState(countdown),
            connector_outcomes={'succeeded'})

        # Submerge
        smach.StateMachine.add(
            'Submerge',
            SetVelocityState.create_set_depth_state(depth, tolerance=0.1))
    return sm


def lane_task(yaw, speed, duration):
    # Returns a state machine which will dead reckon towards the lane and
    # align over it using visual servoing.
    motion_sm = smach.StateMachine(
        ['succeeded', 'lane_not_found', 'preempted', 'aborted'],
        input_keys=['yaw_setpoint', 'depth_setpoint'],
        output_keys=['yaw_setpoint', 'depth_setpoint'])
    with motion_sm:
        motion_sm.add_auto(
            'Switch Entry Point',
            components.switch_entry_point(motion_sm, 'Search Pattern'),
            connector_outcomes=['succeeded'])
        motion_sm.add_auto(
            'Dead Reckon',
            components.dead_reckon_with_monitor(yaw, speed, duration),
            connector_outcomes=['lane_seen'])
        motion_sm.add_auto(
            'Search Pattern', components.search_pattern_with_condition(),
            connector_outcomes=['lane_tracking'])
        motion_sm.add('Visual Servo', components.visual_servo())
    tracking_concurrence = smach.Concurrence(
        ['succeeded', 'lane_not_found', 'tracking_failed',
         'preempted', 'aborted'],
        child_termination_cb=lambda x: True,
        default_outcome='tracking_failed',
        outcome_map={
            'succeeded': {'Motion': 'succeeded'},
            'lane_not_found': {'Motion': 'lane_not_found'},
            'tracking_failed': {'Tracking': 'aborted'}},
        input_keys=['yaw_setpoint', 'depth_setpoint'],
        output_keys=['yaw_setpoint', 'depth_setpoint'])
    with tracking_concurrence:
        tracking_concurrence.add('Motion', motion_sm)
        tracking_concurrence.add(
            'Tracking',
            smach_ros.SimpleActionState(
                'tracking',
                SetCVTargetAction,
                goal=SetCVTargetGoal(CVTarget.LANE)))
    retry_tracking_sm = smach.StateMachine(
        ['succeeded', 'lane_not_found', 'preempted', 'aborted'],
        input_keys=['yaw_setpoint', 'depth_setpoint'],
        output_keys=['yaw_setpoint', 'depth_setpoint'])
    with retry_tracking_sm:
        # On failure we retry. This will cause the robot to execute a search
        # pattern.
        retry_tracking_sm.add(
            'Tracking Concurrence',
            tracking_concurrence,
            transitions={'tracking_failed': 'Tracking Concurrence'})
    sm = smach.Concurrence(
        ['succeeded', 'lane_not_found', 'preempted', 'aborted'],
        child_termination_cb=lambda x: True,
        default_outcome='aborted',
        outcome_map={'succeeded': {'Retry Tracking': 'succeeded'},
                     'lane_not_found': {'Retry Tracking': 'lane_not_found'}},
        input_keys=['yaw_setpoint', 'depth_setpoint'],
        output_keys=['yaw_setpoint', 'depth_setpoint'])
    with sm:
        sm.add('Retry Tracking', retry_tracking_sm)
        sm.add('CV', smach_ros.SimpleActionState(
            'cv_server',
            SetCVTargetAction,
            goal=SetCVTargetGoal(CVTarget.LANE)))
    return sm
