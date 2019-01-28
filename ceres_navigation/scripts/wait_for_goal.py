import rospy
import smach
import threading
from geometry_msgs.msg import PoseStamped


class WaitForGoal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['received_goal', 'preempted'], input_keys=[], output_keys=['target_pose'])
        self.target_pose = PoseStamped()
        self.signal = threading.Event()
        self.subscriber = None

    def execute(self, user_data):

        rospy.loginfo("Waiting for a goal...")
        self.signal.clear()
        self.subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        while not rospy.is_shutdown() and not self.signal.is_set() and not self.preempt_requested():
            rospy.logdebug("Waiting for a goal...")
            self.signal.wait(1)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return 'preempted'

        user_data.target_pose = self.target_pose
        pos = self.target_pose.pose.position
        rospy.loginfo("Received goal pose: (%s, %s, %s)", pos.x, pos.y, pos.z)

        return 'received_goal'

    def goal_callback(self, msg):
        rospy.logdebug("Received goal pose: %s", str(msg))
        self.target_pose = msg
        self.signal.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()