import smach

from planner_sm import PlannerStateMachine


class PlanExecStateMachine(smach.Concurrence):
    _planners = None

    @classmethod
    def set_planners(cls, planners):
        cls._planners = planners

    def __init__(self):
        if self._planners is None:
            raise ValueError("you have to set up planners first by calling PlanExecStateMachine.set_planners([...]).")

        outcome_map = {
            'succeeded': {},
            'failure': {},
            'preempted': {},
            'aborted': {},
        }

        for planner in self._planners:
            outcome_map['succeeded'][planner.upper()] = 'succeeded'
            outcome_map['failure'][planner.upper()] = 'failure'
            outcome_map['preempted'][planner.upper()] = 'preempted'
            outcome_map['aborted'][planner.upper()] = 'aborted'

        smach.Concurrence.__init__(
            self,
            outcomes=['failure', 'succeeded', 'preempted', 'aborted'],
            default_outcome='succeeded',
            outcome_map=outcome_map,
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path'],
            child_termination_cb=self.plan_exec_sm_child_termination_cb,
            outcome_cb=self.plan_exec_sm_outcome_cb)
        with self:
            for index, planner in enumerate(self._planners):
                planner_sm = PlannerStateMachine(index, planner, 'eband')
                smach.Concurrence.add(planner.upper(), planner_sm)

    def plan_exec_sm_child_termination_cb(self, outcome_map):
        # check for success
        for planner in self._planners:
            if outcome_map[planner.upper()] == 'succeeded':
                return True  # terminate concurrent statemachine, if one execution succeeded

        # check, if all planners are aborted or preempted
        aborted_or_preempted = True
        for planner in self._planners:
            print "plan_exec child terminated with mode " + str(outcome_map[planner.upper()])
            if outcome_map[planner.upper()] not in ('preempted', 'aborted'):
                aborted_or_preempted = False
        if aborted_or_preempted:
            return True  # every planner was aborted or preempted. Cancel this statemachine

        # Do not terminate the concurrent statemachine, if at least one child is running.
        return False

    # gets called when ALL child states are terminated
    def plan_exec_sm_outcome_cb(self, outcome_map):
        for planner in self._planners:
            if outcome_map[planner.upper()] == 'succeeded':
                return 'succeeded'

        aborted_or_preempted = True
        aborted_found = False
        for planner in self._planners:
            if outcome_map[planner.upper()] not in ('preempted', 'aborted'):
                aborted_or_preempted = False
            if outcome_map[planner.upper()] == 'aborted':
                aborted_found = True

        if aborted_or_preempted:
            if aborted_found:
                return 'aborted'
            else:
                return 'preempted'

        return 'failure'
