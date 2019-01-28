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
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['path'],
            output_keys=['outcome', 'message', 'final_pose', 'target_dist', 'target_angle']
        )

        with self:

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
                    'aborted': 'aborted'
                }
            )

    @staticmethod
    @smach.cb_interface(input_keys=['path'])
    def controller_goal_cb(user_data, goal):
        goal.path = user_data.path
        goal.controller = 'eband'

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal', 'angle_to_goal'],
        outcomes=['succeeded', 'aborted', 'preempted'])
    def controller_result_cb(user_data, status, result):
        user_data.outcome = result.outcome
        user_data.message = result.message
        user_data.final_pose = result.final_pose
        user_data.dist_to_goal = result.dist_to_goal
        user_data.angle_to_goal = result.angle_to_goal
        if result.outcome == ExePathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == ExePathResult.CANCELED:
            return 'preempted'
        else:
            return 'aborted'


class Planning(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path', 'cost']
        )

        with self:
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
        output_keys=['outcome', 'message', 'path', 'cost'],
        outcomes=['succeeded', 'preempted', 'aborted'])
    def planner_result_cb(user_data, status, result):
        user_data.message = result.message
        user_data.outcome = result.outcome
        user_data.path = result.path
        user_data.cost = result.cost
        if result.outcome == GetPathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        return 'aborted'


class Recovery(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['behavior'],
            output_keys=['outcome', 'message']
        )

        with self:
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
    @smach.cb_interface(input_keys=['behavior'])
    def recovery_goal_cb(user_data, goal):
        goal.behavior = user_data.behavior

    @staticmethod
    @smach.cb_interface(
        output_keys=['outcome', 'message'],
        outcomes=['succeeded', 'aborted', 'preempted'])
    def recovery_result_cb(user_data, status, result):
        if result.outcome == RecoveryResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == RecoveryResult.CANCELED:
            return 'preempted'
        else:
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
            output_keys=['target_pose', 'path'],
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
                    'aborted': 'RECOVERY'
                }
            )

            smach.StateMachine.add(
                'RECOVERY',
                Recovery(),
                transitions={
                    'succeeded': 'PLANNING',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )


class PlanToGoal(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[],
            output_keys=['path'],
        )

        with self:
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
            outcomes=['new_path', 'succeeded', 'preempted'],
            default_outcome='preempted',
            input_keys=['path'],
            output_keys=['path'],
            outcome_map={
                'new_path': {'PLAN_TO_GOAL': 'succeeded'},
                'succeeded': {'CONTROL': 'succeeded'},
                'preempted': {'PLAN_TO_GOAL': 'preempted',
                              'CONTROL': 'preempted'}
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
