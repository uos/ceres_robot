import smach
import smach_ros
import rospy
import math
import tf

from tf.transformations import euler_from_quaternion

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

if hasattr(smach.CBInterface, '__get__'):
    from smach import cb_interface
else:
    from smach_polyfill import cb_interface

# These interfaces are used for the ReplanningStateMachine (down below) and in execute_while_replanning.py
@cb_interface(input_keys=['path'])
def exe_path_goal_cb(userdata, goal):
    goal.path = userdata.path
    goal.controller = 'eband'

@cb_interface(
    output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
    outcomes=['succeeded', 'failure'])
def exe_path_result_cb(userdata, status, result):
    userdata.message = result.message
    userdata.outcome = result.outcome
    userdata.dist_to_goal = result.dist_to_goal
    userdata.final_pose = result.final_pose
    if result.outcome == ExePathResult.SUCCESS:
        return 'succeeded'
    elif result.outcome == ExePathResult.CANCELED:
        return 'preempted'
    else:
        rospy.loginfo('Execution of eband terminated with non-success status code %s:\n%s', str(result.outcome), result.message)
        return 'failure'


class ReplanningStateMachine(smach.StateMachine):
    """
    The actual estimation happens here. Then planning and execution is done.
    """
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=['preempted', 'aborted', 'failure', 'succeeded'],
            input_keys=['target_pose', 'path'],
            output_keys=['outcome', 'message', 'path'])

        self._max_vel_lin = rospy.get_param('/move_base_flex/eband/max_vel_lin')
        self._max_vel_th = rospy.get_param('/move_base_flex/eband/max_vel_th')

        with self:
            state = smach_ros.SimpleActionState(
                'move_base_flex/get_path',
                GetPathAction,
                goal_cb=self.get_path_goal_cb,
                result_cb=self.get_path_result_cb)
            smach.StateMachine.add(
                'SBPLLatticePlanner',
                state,
                transitions={
                    'succeeded': 'Execution',
                    'failure': 'failure',
                    'preempted': 'preempted',
                    'aborted': 'aborted'})

            state = smach_ros.SimpleActionState(
                'move_base_flex/exe_path',
                ExePathAction,
                goal_cb=exe_path_goal_cb,
                result_cb=exe_path_result_cb)
            smach.StateMachine.add(
                'Execution',
                state,
                transitions={
                    'succeeded': 'succeeded',
                    'failure': 'failure',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                })

    @cb_interface(input_keys=['target_pose', 'path'])
    def get_path_goal_cb(self, userdata, goal):
        # What happens here?
        # The SBPLLatticePlanner is started with a different start pose then the robot pose.
        # This pose is one of the poses in the path provided by the GlobalPlanner. We search the pose
        # where the robot may be, if the SBPLLatticePlanner returns. It's calculated by the amount the eband
        # planner will rotate the robot (difference in the robots angle and the angle of the first pose) and
        # the length of the path.
        # Please notice the two magic values in the code, which factor the values into `planning_time`.

        # get current robots pose
        listener = tf.TransformListener()
        ok = False
        lookup_timeout_time = rospy.Time.now() + rospy.Duration(5)
        while not ok:
            if lookup_timeout_time < rospy.Time.now():
                raise Exception("Lookup timed out!")
            try:
                trans, rot = listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
                ok = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        # Calculate the angle between the first pose of the path and the robot
        yaw_start = euler_from_quaternion(rot)[2]
        yaw_start *= (180.0 / math.pi)
        if yaw_start < 0:
            yaw_start += 360.0
        rospy.loginfo("start angle: %s", yaw_start)

        pose = userdata.path.poses[0]
        orientation = pose.pose.orientation
        yaw_pose = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        yaw_pose *= (180.0 / math.pi)
        if yaw_pose < 0:
            yaw_pose += 360.0

        angle_diff = abs(yaw_start - yaw_pose)
        if 360.0 - angle_diff < angle_diff:
            angle_diff = 360.0 - angle_diff
        rospy.loginfo('pose angle: %s', yaw_pose)
        rospy.loginfo('difference: %s', angle_diff)

        # Wait one second for a half turn
        rotation_time = angle_diff/180.0  # MAGIC 1
        rospy.loginfo(self._max_vel_th)
        look_forward_length = rotation_time * self._max_vel_th

        #calculate length of path
        poses = userdata.path.poses
        path_length = 0
        for i in range(len(poses)-1):
            path_length += math.sqrt(
                  pow(poses[i+1].pose.position.x - poses[i].pose.position.x, 2)
                + pow(poses[i+1].pose.position.y - poses[i].pose.position.y, 2))
        rospy.loginfo('Path length: %s', path_length)

        # Add the path length to the planner time.
        translation_time = path_length * 0.1  # MAGIC 2

        # How much the robot can move in the time needed by the planner?
        look_forward_length += translation_time * self._max_vel_lin
        rospy.loginfo('look forward length: %s', look_forward_length)

        #find the point, that is look_forward_length meters away from the start of the path
        if look_forward_length >= path_length:
            pose = poses[len(poses)/2]
            rospy.loginfo('forward length is greater then the path length. Take the middle point to plan.')
        else:
            check_dist = 0
            i = 0
            while check_dist < look_forward_length and i < len(poses) - 1:
                check_dist += math.sqrt(
                    pow(poses[i+1].pose.position.x - poses[i].pose.position.x, 2)
                    + pow(poses[i+1].pose.position.y - poses[i].pose.position.y, 2))
                i += 1
            pose = poses[i]
            rospy.loginfo(i, ' / ', len(poses))

        goal.use_start_pose = True
        goal.start_pose = pose
        goal.tolerance = 0.2
        goal.target_pose = userdata.target_pose
        goal.planner = 'SBPLLatticePlanner'

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
            rospy.logwarn('Planning with SBPLLatticePlanner terminated with non-success status code %s:\n%s', str(result.outcome), result.message)
            return 'failure'