#!/usr/bin/env python
import rospy
import smach
import smach_ros

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult

from wait_for_goal import WaitForGoal
from geometry_msgs.msg import PoseStamped

from smach_utils import cb_interface
from plan_exec_sm import PlanExecStateMachine


class MBFStateMachine(smach.StateMachine):
    _recovery_behaviors = None

    @classmethod
    def set_recovery_behaviors(cls, recovery_behaviors):
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
                    'succeeded': 'PLAN_EXEC',
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


SM = None
SM_target_pose = None


def goal_callback(target_pose):
    if SM is not None and SM.get_active_states()[0] != 'WAIT_FOR_GOAL':
        SM.request_preempt()
        global SM_target_pose
        SM_target_pose = target_pose


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('mbf_state_machine')        

        planners = [entry['name'] for entry in rospy.get_param('/move_base_flex/planners')]
        if len(planners) == 0:
            raise ValueError('You have to specify at least one planner')
        PlanExecStateMachine.set_planners(planners)


        subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)


        recovery_behaviors = [entry['name'] for entry in rospy.get_param('/move_base_flex/recovery_behaviors')]
        MBFStateMachine.set_recovery_behaviors(recovery_behaviors)

        SM = MBFStateMachine()

        sis = smach_ros.IntrospectionServer('mbf_state_machine_server', SM, '/MBF_SM')
        sis.start()

        if SM_target_pose is not None:
            SM.userdata._data = {}
            SM.userdata.recovery_behavior_index = 0

            target_userdata = smach.UserData()
            target_userdata.target_pose = SM_target_pose
            SM.set_initial_state(['PLAN_EXEC'], userdata=target_userdata)
            SM_target_pose = None
        outcome = SM.execute()

        subscriber.unregister()
        sis.stop()

