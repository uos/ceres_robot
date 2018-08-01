import smach
import smach_ros

from mbf_msgs.msg import ExePathAction

from replan_sm import *


class ExecWhileReplanStateMachine(smach.Concurrence):

    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['failure', 'succeeded', 'preempted', 'aborted'],
            default_outcome='succeeded',
            outcome_map = {
                'succeeded': {
                    'Execution': 'succeeded',
                    'SBPLLatticePlanner': 'succeeded',

                },
                'failure': {
                    'Execution': 'failure',
                    'SBPLLatticePlanner': 'failure',

                },
                'preempted': {
                    'Execution': 'preempted',
                    'SBPLLatticePlanner': 'preempted',


                },
                'aborted': {
                    'Execution': 'aborted',
                    'SBPLLatticePlanner': 'aborted',
                },
            },
            input_keys=['target_pose', 'path'],
            output_keys=['outcome', 'message', 'path'],
            child_termination_cb=self.plan_exec_sm_child_termination_cb,
            outcome_cb=self.plan_exec_sm_outcome_cb)

        with self:
            smach.Concurrence.add(
                'SBPLLatticePlanner',
                ReplanningStateMachine())

            state = smach_ros.SimpleActionState(
                'move_base_flex/exe_path',
                ExePathAction,
                goal_cb=exe_path_goal_cb,
                result_cb=exe_path_result_cb)
            smach.Concurrence.add(
                'Execution',
                state)

    def plan_exec_sm_child_termination_cb(self, outcome_map):
        # check for success
        for label in ('Execution', 'SBPLLatticePlanner'):
            if outcome_map[label] == 'succeeded':
                return True
        return False

    # gets called when ALL child states are terminated
    def plan_exec_sm_outcome_cb(self, outcome_map):
        print "plan_exec_sm_outcome_cb"
        print outcome_map

        for label in ('Execution', 'SBPLLatticePlanner'):
            if outcome_map[label] == 'succeeded':
                return 'succeeded'

        aborted_or_preempted = True
        aborted_found = False
        for label in ('Execution', 'SBPLLatticePlanner'):
            if outcome_map[label] not in ('preempted', 'aborted'):
                aborted_or_preempted = False
            if outcome_map[label] == 'aborted':
                aborted_found = True

        if aborted_or_preempted:
            if aborted_found:
                return 'aborted'
            else:
                return 'preempted'

        return 'failure'