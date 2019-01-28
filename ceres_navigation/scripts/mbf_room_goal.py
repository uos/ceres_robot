#!/usr/bin/env python
import rospy
import smach
import smach_ros

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult

from wait_for_goal import WaitForGoal
from geometry_msgs.msg import PoseStamped

from plan_exec_sm import PlanExecStateMachine
from monte_carlo_poses import GetPoses
from get_best_target_pose import GetBestTargetPose

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface


class MBFStateMachine(smach.StateMachine):
    """
    A Statemachine accepting goals and checks, if the goal is in a room (see monte_carlo_poses.py).
    If a room is found, SOme poses are randomly put into it and the best target pose is searched. Then
    the plan_exec_sm will navigate to it. If the pose isn't in any room, it will just be taken as the target pose.
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

        with self:  # Just simple plug and play with the states.
            smach.StateMachine.add(
                'WAIT_FOR_GOAL',
                WaitForGoal(),
                transitions={
                    'succeeded': 'GET_POSES',
                    'preempted': 'preempted'})

            # This state takes a target_pose, lookup the room. If a room is found, it will put an array
            # of poses into the userdata
            smach.StateMachine.add(
                'GET_POSES',
                GetPoses(),
                transitions={
                    'succeeded': 'GET_BEST_TARGET_POSE',
                    'room_not_found': 'PLAN_EXEC',
                    'failure': 'WAIT_FOR_GOAL',
                    'preempted': 'preempted'})

            # Takes an array of poses and planns with the GlobalPlanner parallel to every target_pose
            # Sets the target_pose in the userdata as the best target pose
            smach.StateMachine.add(
                'GET_BEST_TARGET_POSE',
                GetBestTargetPose(),
                transitions={
                    'succeeded': 'PLAN_EXEC',
                    'failure': 'WAIT_FOR_GOAL',
                    'preempted': 'preempted'})

            smach.StateMachine.add(
                'RECOVERY',
                smach_ros.SimpleActionState(
                    'move_base_flex/recovery',
                    RecoveryAction,
                    goal_cb=self.recovery_goal_cb,
                    result_cb=self.recovery_result_cb),
                transitions={
                    'succeeded': 'PLAN_EXEC',
                    'failure': 'WAIT_FOR_GOAL'})

            plan_exec_sm = PlanExecStateMachine()
            smach.StateMachine.add(
                'PLAN_EXEC',
                plan_exec_sm,
                transitions={
                    'failure': 'RECOVERY',
                    'succeeded': 'WAIT_FOR_GOAL',
                    'invalid': 'WAIT_FOR_GOAL'})

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
        if result.outcome == RecoveryResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == RecoveryResult.CANCELED:
            return 'preempted'
        else:
            return 'failure'


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('mbf_state_machine')

        planners = [entry['name'] for entry in rospy.get_param('/move_base_flex/planners')]
        if len(planners) == 0:
            raise ValueError('You have to specify at least one planner')
        PlanExecStateMachine.set_planners(planners)

        recovery_behaviors = [entry['name'] for entry in rospy.get_param('/move_base_flex/recovery_behaviors')]
        MBFStateMachine.set_recovery_behaviors(recovery_behaviors)

        SM = MBFStateMachine()

        sis = smach_ros.IntrospectionServer('mbf_state_machine_server', SM, '/MBF_SM')
        sis.start()

        outcome = SM.execute()
        sis.stop()
        rospy.spin()