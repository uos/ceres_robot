import math
import smach
import smach_ros

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from monte_carlo_poses import NUM_POSES

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface


class GetBestTargetPose(smach.Concurrence):
    def __init__(self):
        outcome_map = {
            'succeeded': {},
            'failure': {},
            'preempted': {},
            'aborted': {},
        }

        for i in range(NUM_POSES):
            i = str(i)
            outcome_map['succeeded'][i] = 'succeeded'
            outcome_map['failure'][i] = 'failure'
            outcome_map['preempted'][i] = 'preempted'
            outcome_map['aborted'][i] = 'aborted'

        smach.Concurrence.__init__(
            self,
            outcomes=['failure', 'succeeded', 'preempted', 'aborted'],
            default_outcome='succeeded',
            outcome_map=outcome_map,
            input_keys=['poses'],
            output_keys=['target_pose'],
            child_termination_cb=self.get_best_target_pose_sm_child_termination_cb,
            outcome_cb=self.get_best_target_pose_sm_outcome_cb)
        with self:
            for i in range(NUM_POSES):
                state = smach_ros.SimpleActionState(
                    'move_base_flex/get_path',
                    GetPathAction,
                    goal_cb=self.get_path_goal_cb(i),
                    result_cb=self.get_path_result_cb(i))
                smach.Concurrence.add(str(i), state)

    def execute(self, parent_ud = smach.UserData()):
        self._min_path_length = 1e10
        self._best_pose = None

        return smach.Concurrence.execute(self, parent_ud=parent_ud)

    def get_best_target_pose_sm_child_termination_cb(self, outcome_map):
        # Always wait for every child to terminate
        return False

    # gets called when ALL child states are terminated
    def get_best_target_pose_sm_outcome_cb(self, outcome_map):
        if self._best_pose is None:
            return 'failure'

        print 'Min path length: ', self._min_path_length
        return 'succeeded'

    def get_path_goal_cb(self, index):
        @cb_interface(input_keys=['poses'])
        def __cb(userdata, goal):
            self._poses = userdata.poses

            goal.use_start_pose = False
            goal.tolerance = 0.2
            goal.target_pose = userdata.poses[index]
            goal.planner = 'GlobalPlanner'
            goal.concurrency_slot = index
        return __cb


    def get_path_result_cb(self, index):
        @cb_interface(
            input_keys=['poses'],
            output_keys=['target_pose'],
            outcomes=['succeeded', 'failure'])
        def __cb(userdata, status, result):

            #calculate length of path
            poses = result.path.poses
            path_length = 0
            for i in range(len(poses)-1):
                path_length += math.sqrt(
                    pow(poses[i+1].pose.position.x - poses[i].pose.position.x, 2)
                    + pow(poses[i+1].pose.position.y - poses[i].pose.position.y, 2))

            # Check, if this path is the shortest
            if path_length < self._min_path_length:
                self._min_path_length = path_length
                self._best_pose = userdata.poses[index]
                userdata.target_pose = userdata.poses[index]

            if result.outcome == GetPathResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == GetPathResult.CANCELED:
                return 'preempted'
            else:
                print 'Planning with GlobalPlanner terminated with non-success status code %s:\n%s' % (str(result.outcome), result.message)
                return 'failure'
        return __cb
