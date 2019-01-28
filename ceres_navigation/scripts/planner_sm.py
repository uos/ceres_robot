import smach
import smach_ros

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface


class PlannerStateMachine(smach.StateMachine):
    """
    A simple state machine, that plans globally with the given planner in the given concurrency_slot
    and after planning it will be executed with the controller given.
    """
    def __init__(self, concurrency_slot, planner_name, controller_name):
        smach.StateMachine.__init__(
            self,
            outcomes=['preempted', 'succeeded', 'aborted', 'failure', 'invalid'],
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path'])

        self._concurrency_slot = concurrency_slot
        self._planner_name = planner_name
        self._controller_name = controller_name

        with self:
            state = smach_ros.SimpleActionState(
                'move_base_flex/get_path',
                GetPathAction,
                goal_cb=self.get_path_goal_cb,
                result_cb=self.get_path_result_cb)
            smach.StateMachine.add(
                planner_name.upper(),
                state,
                transitions={
                    'succeeded': planner_name.upper()+'_EXEC',
                    'failure': 'failure',
                    'invalid': 'invalid',
                    'preempted': 'preempted'})

            state = smach_ros.SimpleActionState(
                'move_base_flex/exe_path',
                ExePathAction,
                goal_cb=self.exe_path_goal_cb,
                result_cb=self.exe_path_result_cb)
            smach.StateMachine.add(
                planner_name.upper() + '_EXEC',
                state,
                transitions={
                    'succeeded': 'succeeded',
                    'failure': 'failure',
                    'preempted': 'preempted'})

    @cb_interface(input_keys=['target_pose'])
    def get_path_goal_cb(self, userdata, goal):
        goal.use_start_pose = False
        goal.tolerance = 0.2
        goal.target_pose = userdata.target_pose
        goal.planner = self._planner_name
        goal.concurrency_slot = self._concurrency_slot

    @cb_interface(
        output_keys=['message', 'outcome', 'path'],
        outcomes=['succeeded', 'failure', 'preempted', 'invalid'])
    def get_path_result_cb(self, userdata, status, result):
        if result is None:  # something preempted or aborted this
            print 'result is None!'
            return 'aborted'

        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path

        if result.outcome == GetPathResult.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPathResult.CANCELED:
            return 'preempted'
        elif result.outcome in (GetPathResult.INVALID_START, GetPathResult.INVALID_GOAL, GetPathResult.NO_PATH_FOUND, GetPathResult.PAT_EXCEEDED):
            # We will take PAT_EXCEEDED as a failure to get a path. The most planners currently won't give a better feedback, so if it
            # takes too long to find a path, there might be no path.
            print 'Planning with %s could not get any path %s:\n%s' % (self._planner_name, str(result.outcome), result.message)
            return 'invalid'
        else:
            print 'Planning with %s terminated with non-success status code %s:\n%s' % (self._planner_name, str(result.outcome), result.message)
            return 'failure'

    @cb_interface(input_keys=['path'])
    def exe_path_goal_cb(self, userdata, goal):
        goal.path = userdata.path
        goal.controller = self._controller_name

    @cb_interface(
        output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
        outcomes=['succeeded', 'failure'])
    def exe_path_result_cb(self, userdata, status, result):
        if result:
            userdata.message = result.message
            userdata.outcome = result.outcome
            userdata.dist_to_goal = result.dist_to_goal
            userdata.final_pose = result.final_pose
            if result.outcome == ExePathResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == ExePathResult.CANCELED:
                return 'preempted'
            else:
                print 'Execution of %s terminated with non-success status code %s:\n%s' % (self._planner_name, str(result.outcome), result.message)
                return 'failure'
        else:
            print 'No result, received status: %s' % status
            return 'failure'
