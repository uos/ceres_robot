#!/usr/bin/env python
import rospy
import smach
import smach_ros

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from wait_for_goal import WaitForGoal
from execute_while_replan_sm import ExecWhileReplanStateMachine

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface


class MBFFuturePlanning(smach.StateMachine):
    """
    This statemachine first plans with the GlobalPlanner, then estimates how long the planning
    with the SBPLLatticePlanner takes. Then it plans from a point on the global path, that the robot
    can reach in the estimated time.
    The goal is to make a smooth transition from one global path to another.
    """
    _recovery_behaviors = None

    @classmethod
    def set_recovery_behaviors(cls, recovery_behaviors):
        """Sets the recovery behaviors used by the statemachine"""
        cls._recovery_behaviors = recovery_behaviors

    def __init__(self):
        if self._recovery_behaviors is None:
            raise ValueError("you have to set up planners first by calling MBFStateMachine.set_recovery_behaviors([...]).")

        smach.StateMachine.__init__(self, outcomes=['preempted', 'aborted'])
        self.userdata.recovery_behavior_index = 0  # start with first recovery behavior

        with self:
            smach.StateMachine.add(
                'WAIT_FOR_GOAL',
                WaitForGoal(),
                transitions={
                    'succeeded': 'GlobalPlanner',
                    'preempted': 'preempted'})

            smach.StateMachine.add(
                'RECOVERY',
                smach_ros.SimpleActionState(
                    'move_base_flex/recovery',
                    RecoveryAction,
                    goal_cb=self.recovery_goal_cb,
                    result_cb=self.recovery_result_cb),
                transitions={
                    'succeeded': 'GlobalPlanner',
                    'failure': 'WAIT_FOR_GOAL'})

            # The first get-path state to get the initial path
            state = smach_ros.SimpleActionState(
                'move_base_flex/get_path',
                GetPathAction,
                goal_cb=self.get_path_goal_cb,
                result_cb=self.get_path_result_cb)
            smach.StateMachine.add(
                'GlobalPlanner',
                state,
                transitions={
                    'succeeded': 'EXECUTION',
                    'failure': 'WAIT_FOR_GOAL',
                    'preempted': 'preempted'})

            # Here the "magic" happens. The initial path will be executed while the SBPLLatticePlanner plans a better path.
            exec_while_replan_sm = ExecWhileReplanStateMachine()
            smach.StateMachine.add(
                'EXECUTION',
                exec_while_replan_sm,
                transitions={
                    'failure': 'RECOVERY',
                    'succeeded': 'WAIT_FOR_GOAL'})

    @cb_interface(input_keys=['recovery_behavior_index'], output_keys=['recovery_behavior_index'])
    def recovery_goal_cb(self, userdata, goal):
        # TODO implement a more clever way to call the right behavior. Currently cycles through all behaviors
        behavior = self._recovery_behaviors[userdata.recovery_behavior_index]
        print 'RECOVERY BEHAVIOR:', behavior
        goal.behavior = behavior
        userdata.recovery_behavior_index += 1
        if userdata.recovery_behavior_index >= len(self._recovery_behaviors):
            userdata.recovery_behavior_index = 0

    @cb_interface(output_keys=['outcome', 'message'], outcomes=['succeeded', 'failure'])
    def recovery_result_cb(self, userdata, status, result):
        print result.outcome
        # TODO: preempted and aborted
        if result.outcome == RecoveryResult.SUCCESS:
            return 'succeeded'
        else:
            return 'failure'

    @cb_interface(input_keys=['target_pose'])
    def get_path_goal_cb(self, userdata, goal):
        goal.use_start_pose = False
        goal.tolerance = 0.2
        goal.target_pose = userdata.target_pose
        goal.planner = 'GlobalPlanner'

    @cb_interface(
        output_keys=['message', 'outcome', 'path'],
        outcomes=['succeeded', 'failure'])
    def get_path_result_cb(self, userdata, status, result):
        if result is None:  # something preempted or aborted this
            return 'aborted'

        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path

        if result.outcome == GetPathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        else:
            print 'Planning with GlobalPlanner terminated with non-success status code %s:\n%s' % (str(result.outcome), result.message)
            return 'failure'


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('mbf_state_machine')

        recovery_behaviors = [entry['name'] for entry in rospy.get_param('/move_base_flex/recovery_behaviors')]
        MBFFuturePlanning.set_recovery_behaviors(recovery_behaviors)

        SM = MBFFuturePlanning()

        sis = smach_ros.IntrospectionServer('mbf_state_machine_server', SM, '/MBF_SM')
        sis.start()

        outcome = SM.execute()
        sis.stop()