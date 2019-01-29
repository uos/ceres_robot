#!/usr/bin/env python

import rospy
import smach
import smach_ros
from mbf_msgs.msg import ExePathAction, ExePathResult
from mbf_msgs.msg import GetPathAction, GetPathResult
from mbf_msgs.msg import RecoveryAction, RecoveryResult
from wait_for_goal import WaitForGoal


class Control(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted', 'failed'],
            input_keys=['path'],
            output_keys=[
                'outcome', 'message',
                'final_pose', 'dist_to_goal', 'angle_to_goal',
                'recovery_behavior']
        )

        with self:

            self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'EXE_PATH',
                smach_ros.SimpleActionState(
                    'move_base_flex/exe_path',
                    ExePathAction,
                    goal_cb=Control.controller_goal_cb,
                    result_cb=Control.controller_result_cb),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted',
                    'failed': 'failed',
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['path'])
    def controller_goal_cb(user_data, goal):
        goal.path = user_data.path
        goal.controller = 'dwa'

    @staticmethod
    @smach.cb_interface(
        output_keys=['path', 'outcome', 'message', 'final_pose', 'dist_to_goal', 'angle_to_goal', 'recovery_behavior'],
        outcomes=['succeeded', 'aborted', 'failed', 'preempted'])
    def controller_result_cb(user_data, status, result):
        outcome_map = {
            ExePathResult.COLLISION: 'COLLISION',
            ExePathResult.CANCELED: 'CANCELED',
            ExePathResult.BLOCKED_PATH: 'BLOCKED_PATH',
            ExePathResult.FAILURE: 'FAILURE',
            ExePathResult.INTERNAL_ERROR: 'INTERNAL_ERROR',
            ExePathResult.INVALID_PATH: 'INVALID_PATH',
            ExePathResult.MISSED_GOAL: 'MISSED_GOAL',
            ExePathResult.INVALID_PLUGIN: 'INVALID_PLUGIN',
            ExePathResult.MISSED_PATH: 'MISSED_PATH',
            ExePathResult.NO_VALID_CMD: 'NO_VALID_CMD',
            ExePathResult.NOT_INITIALIZED: 'NOT_INITIALIZED',
            ExePathResult.OSCILLATION: 'OSCILLATION',
            ExePathResult.PAT_EXCEEDED: 'PAT_EXCEEDED',
            ExePathResult.ROBOT_STUCK: 'ROBOT_SUCK',
            ExePathResult.TF_ERROR: 'TF_ERROR',
            ExePathResult.SUCCESS: 'SUCCESS',
        }

        controller_aborted_map = [
            ExePathResult.TF_ERROR,
            ExePathResult.INTERNAL_ERROR,
            ExePathResult.INVALID_PATH,
            ExePathResult.NOT_INITIALIZED,
        ]

        controller_failed_map = [
            ExePathResult.PAT_EXCEEDED,
            ExePathResult.BLOCKED_PATH,
            ExePathResult.FAILURE,
            ExePathResult.MISSED_PATH,
            ExePathResult.MISSED_GOAL,
            ExePathResult.NO_VALID_CMD,
            ExePathResult.OSCILLATION,
            ExePathResult.ROBOT_STUCK,
        ]

        user_data.outcome = result.outcome
        user_data.message = result.message
        user_data.final_pose = result.final_pose
        user_data.dist_to_goal = result.dist_to_goal
        user_data.angle_to_goal = result.angle_to_goal

        recovery_behavior = 'clear_costmap'
        if result.outcome == ExePathResult.SUCCESS:
            p = result.final_pose.pose.position
            rospy.loginfo("Controller arrived at goal: (%s), %s, %s",
                          str(p), outcome_map[result.outcome], result.message)
            return 'succeeded'
        elif result.outcome == ExePathResult.CANCELED:
            rospy.loginfo("Controller has been canceled.")
            return 'preempted'
        elif result.outcome in controller_failed_map:
            rospy.logwarn("Controller failed: %s, %s", outcome_map[result.outcome], result.message)
            user_data.recovery_behavior = recovery_behavior
            rospy.loginfo("Set recovery behavior to %s", recovery_behavior)
            return 'failed'
        else:
            rospy.logfatal("Controller aborted: %s, %s", outcome_map[result.outcome], result.message)
            return 'aborted'


class Planning(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path', 'cost', 'recovery_behavior']
        )

        with self:

            self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'GET_PATH',
                smach_ros.SimpleActionState(
                    'move_base_flex/get_path',
                    GetPathAction,
                    goal_cb=Planning.planner_goal_cb,
                    result_cb=Planning.planner_result_cb),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['target_pose'])
    def planner_goal_cb(user_data, goal):
        goal.use_start_pose = False
        goal.tolerance = 0.2  # 20cm tolerance to the target
        goal.target_pose = user_data.target_pose
        goal.planner = 'SBPLLatticePlanner'  # name of the planner to call see move base flex planners.yaml config

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'path', 'cost', 'recovery_behavior'],
        outcomes=['succeeded', 'preempted', 'aborted'])
    def planner_result_cb(user_data, status, result):
        user_data.message = result.message
        user_data.outcome = result.outcome
        user_data.path = result.path
        user_data.cost = result.cost
        recovery_behavior = 'clear_costmap'
        if result.outcome == GetPathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        else:
            user_data.recovery_behavior = recovery_behavior
            rospy.loginfo("Set recovery behavior to %s", recovery_behavior)
            return 'aborted'


class Recovery(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['recovery_behavior'],
            output_keys=['outcome', 'message', 'recovery_behavior']
        )

        with self:

            self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'RECOVERY',
                smach_ros.SimpleActionState('move_base_flex/recovery',
                                            RecoveryAction,
                                            goal_cb=Recovery.recovery_goal_cb,
                                            result_cb=Recovery.recovery_result_cb),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['recovery_behavior'])
    def recovery_goal_cb(user_data, goal):
        rospy.loginfo("Recovery behavior: %s", user_data.recovery_behavior)
        goal.behavior = user_data.recovery_behavior

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'recovery_behavior'],
        outcomes=['succeeded', 'aborted', 'preempted'])
    def recovery_result_cb(user_data, status, result):
        if result.outcome == RecoveryResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == RecoveryResult.CANCELED:
            return 'preempted'
        else:
            user_data.recovery_behavior = 'rotate_recovery'
            return 'aborted'


class Replanning(smach.Concurrence):

    @staticmethod
    def child_term_cb(outcome_map):
        return True

    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['new_goal', 'succeeded', 'preempted', 'aborted'],
            default_outcome='aborted',
            input_keys=['target_pose'],
            output_keys=['target_pose', 'path', 'recovery_behavior'],
            outcome_map={
                'new_goal': {'WAIT_FOR_GOAL': 'received_goal'},
                'succeeded': {'PLANNING': 'succeeded'},
                'preempted': {'PLANNING': 'preempted',
                              'WAIT_FOR_GOAL': 'preempted'}
            },
            child_termination_cb=Replanning.child_term_cb,
        )

        with self:
            smach.Concurrence.add('PLANNING', Planning())
            smach.Concurrence.add('WAIT_FOR_GOAL', WaitForGoal())


class NavigationStateMachine(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['target_pose'],
            output_keys=['outcome', 'message']
        )

        with self:

            smach.StateMachine.add(
                'PLANNING',
                Planning(),
                transitions={
                    'succeeded': 'CONTROL',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )

            smach.StateMachine.add(
                'CONTROL',
                Control(),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted',
                    'failure': 'RECOVERY',
                }
            )

            smach.StateMachine.add(
                'RECOVERY',
                Recovery(),
                transitions={
                    'succeeded': 'PLANNING',
                    'preempted': 'preempted',
                    'aborted': 'aborted',
                }
            )


class PlanToGoal(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[],
            output_keys=['path', 'recovery_behavior', 'target_pose'],
        )

        with self:

            #self.userdata.path = None
            #self.userdata.recovery_behavior = None

            smach.StateMachine.add(
                'WAIT_FOR_GOAL',
                WaitForGoal(),
                transitions={
                    'received_goal': 'REPLANNING',
                    'preempted': 'preempted',
                })
            smach.StateMachine.add(
                'REPLANNING',
                Replanning(),
                transitions={
                    'new_goal': 'REPLANNING',
                    'succeeded': 'succeeded',
                    'preempted': 'preempted',
                    'aborted': 'aborted',   # TODO recovery
                })


class NavigateToGoal(smach.Concurrence):

    @staticmethod
    def child_term_cb(outcome_map):
        return True

    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['new_path', 'succeeded', 'preempted', 'failed'],
            default_outcome='preempted',
            input_keys=['path', 'recovery_behavior'],
            output_keys=['target_pose', 'recovery_behavior', 'path'],
            outcome_map={
                'new_path': {'PLAN_TO_GOAL': 'succeeded'},
                'succeeded': {'CONTROL': 'succeeded'},
                'preempted': {'PLAN_TO_GOAL': 'preempted',
                              'CONTROL': 'preempted'},
                'failed': {'CONTROL': 'failed'}
            },
            child_termination_cb=NavigateToGoal.child_term_cb,
        )

        with self:
            smach.Concurrence.add('PLAN_TO_GOAL', PlanToGoal())
            smach.Concurrence.add('CONTROL', Control())


def main():
    rospy.init_node('navigation')

    base_sm = smach.StateMachine(outcomes=['preempted', 'aborted'])

    with base_sm:
        smach.StateMachine.add(
            'PLAN_TO_GOAL',
            PlanToGoal(),
            transitions={
                'succeeded': 'NAVIGATE_TO_GOAL',
                'preempted': 'preempted',
                'aborted': 'aborted'
            }
        )

        smach.StateMachine.add(
            'NAVIGATE_TO_GOAL',
            NavigateToGoal(),
            transitions={
                'succeeded': 'PLAN_TO_GOAL',
                'preempted': 'preempted',
                'new_path': 'NAVIGATE_TO_GOAL',
                'failed': 'RECOVERY'
            }
        )

        smach.StateMachine.add(
            'RECOVERY',
            Recovery(),
            transitions={
                'succeeded': 'REPLANNING',
                'preempted': 'preempted',
                'aborted': 'RECOVERY',  # TODO solve recovery loop
            }
        )

        smach.StateMachine.add(
            'REPLANNING',
            Replanning(),
            transitions={
                'new_goal': 'REPLANNING',
                'succeeded': 'NAVIGATE_TO_GOAL',
                'preempted': 'preempted',
                'aborted': 'RECOVERY',  # davide into aborted and failed, more specific failure code and recovery
            }
        )

    sis = smach_ros.IntrospectionServer('navigation', base_sm, '/NAVIGATION')
    sis.start()
    outcome = base_sm.execute()
    rospy.loginfo("Navigation smach outcome: %s", outcome)

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
