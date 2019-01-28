import smach

from planner_sm import PlannerStateMachine


class PlanExecStateMachine(smach.Concurrence):
    _planners = None

    @classmethod
    def set_planners(cls, planners):
        """Set all global planners usd by the statemachine."""
        cls._planners = planners

    def __init__(self):
        if self._planners is None:
            raise ValueError("you have to set up planners first by calling PlanExecStateMachine.set_planners([...]).")

        outcome_map = {
            'succeeded': {},
            'failure': {},
            'invalid': {},
            'preempted': {},
            'aborted': {},
        }

        # Map all outcomes from all children to this container
        for planner in self._planners:
            outcome_map['succeeded'][planner.upper()] = 'succeeded'
            outcome_map['failure'][planner.upper()] = 'failure'
            outcome_map['invalid'][planner.upper()] = 'invalid'
            outcome_map['preempted'][planner.upper()] = 'preempted'
            outcome_map['aborted'][planner.upper()] = 'aborted'

        smach.Concurrence.__init__(
            self,
            outcomes=['failure', 'invalid', 'succeeded', 'preempted', 'aborted'],
            default_outcome='succeeded',
            outcome_map=outcome_map,
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path'],
            child_termination_cb=self.plan_exec_sm_child_termination_cb,
            outcome_cb=self.plan_exec_sm_outcome_cb)
        # Add a planner statemachine for every planner specified
        with self:
            for index, planner in enumerate(self._planners):
                planner_sm = PlannerStateMachine(index, planner, 'eband')
                smach.Concurrence.add(planner.upper(), planner_sm)

    def plan_exec_sm_child_termination_cb(self, outcome_map):
        # check for success
        for planner in self._planners:
            if outcome_map[planner.upper()] == 'succeeded':
                return True  # terminate concurrent statemachine, if one execution succeeded

        # check, if all planners are aborted, preempted or invalid
        aborted_preempted_invalid = True
        for planner in self._planners:
            rospy.logwarn("plan_exec child terminated with outcome %s", str(outcome_map[planner.upper()]))
            if outcome_map[planner.upper()] not in ('preempted', 'aborted', 'invalid'):
                aborted_preempted_invalid = False
        if aborted_preempted_invalid:
            return True  # every planner was aborted or preempted. Cancel this statemachine

        # Do not terminate the concurrent statemachine, if at least one child is running.
        return False

    # gets called when ALL child states are terminated
    def plan_exec_sm_outcome_cb(self, outcome_map):
        # If at least one execution succeeded, it's a success!
        for planner in self._planners:
            if outcome_map[planner.upper()] == 'succeeded':
                return 'succeeded'

        # Check, if every planner is invalid
        invalid = True
        for planner in self._planners:
            if outcome_map[planner.upper()] != 'invalid':
                invalid = False
        if invalid:
            print 'No path was found.'
            return 'invalid'

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

        # If nothing above is right, the execution and planning fails
        return 'failure'