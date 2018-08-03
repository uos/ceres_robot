import rospy
import smach
import random

from mbf_msgs.srv import CheckPose, CheckPoseRequest, CheckPoseResponse

from geometry_msgs.msg import PoseStamped, PoseArray

# Those are rectangles given by two points: p1 is the upper left one, p2 right down.
ROOMS = {
    #    (( p1.x, p1.y ), ( p2.x, p2.y ))
    #'A': ((1.13393211365, 5.49522399902), (8.10231494904, -0.412775993347)),
    #'B': ((2.38570976257, -0.625261306763), (8.14372158051, -3.99622440338)),
    'unten-rechts': ((3.43812513351, 2.89922595024), (5.91117191315, 0.375998139381))
}
NUM_POSES = 10


class GetPoses(smach.State):
    """
    Takes a target_pose, lookup the room (see above) and randomly throws NUM_POSES poses into it.
    These poses are checked to be free. Published a pose array to /poses_checked.
    """

    _poses_pub = None  # Publisher for the poses

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'room_not_found', 'failure', 'preempted'],
            input_keys=['target_pose'],
            output_keys=['poses'])

        if GetPoses._poses_pub is None:
            GetPoses._poses_pub = rospy.Publisher('poses_checked', PoseArray, queue_size=10)

    def execute(self, userdata):
        room = self.get_room(userdata.target_pose)
        if room is None:
            print 'The given pose does not lay in any room! Just navigating to it..'
            return 'room_not_found'

        rospy.wait_for_service('/move_base_flex/check_pose_cost')

        poses = []
        while len(poses) < NUM_POSES:
            # Construct a pose
            x, y = self.get_rand_xy_for_room(room)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = userdata.target_pose.pose.orientation
            try:
                # check it. If it's free, append to poses
                check_pose_cost = rospy.ServiceProxy('/move_base_flex/check_pose_cost', CheckPose)
                request = CheckPoseRequest()
                request.pose = pose
                request.safety_dist = 0.1
                request.costmap = CheckPoseRequest.GLOBAL_COSTMAP
                request.current_pose = False

                res = check_pose_cost(request)
                if res.state == CheckPoseResponse.FREE:
                    poses.append(pose)
            except rospy.ServiceException, e:
                print 'Service call failed: %s' % str(e)
                return 'failure'
        userdata.poses = poses

        # convert to PoseArray for visualization and publish it
        pose_array = PoseArray()
        pose_array.header = poses[0].header
        for pose in poses:
            pose_array.poses.append(pose.pose)
        self._poses_pub.publish(pose_array)

        if rospy.is_shutdown():
            return 'preempted'
        return 'succeeded'

    def get_room(self, pose):
        pos = pose.pose.position
        for room, rect in ROOMS.items():
            if pos.x >= rect[0][0] and pos.x <= rect[1][0] and pos.y <= rect[0][1] and pos.y >= rect[1][1]:
                return room
        return None

    def get_rand_xy_for_room(self, room):
        x = random.random() * (ROOMS[room][1][0] - ROOMS[room][0][0]) + ROOMS[room][0][0]
        y = random.random() * (ROOMS[room][0][1] - ROOMS[room][1][1]) + ROOMS[room][1][1]
        return x, y
