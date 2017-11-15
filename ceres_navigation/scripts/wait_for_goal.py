import rospy
import smach
import time

from geometry_msgs.msg import PoseStamped


class WaitForGoal(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted'],
                             input_keys=['start_pose', 'waypoints', 'tolerance', 'use_start_pose'],
                             output_keys=['start_pose', 'target_pose', 'waypoints', 'tolerance', 'use_start_pose'])
        self.global_target_pose = PoseStamped()
        self.subscriber = None
        self.flag = 0

    def execute(self, userdata):
        self.flag = 0
        rate = 0.3
        self.subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        while not self.flag and not rospy.is_shutdown():    
            time.sleep(rate)

        userdata.target_pose = self.global_target_pose
        print "Target Pose:", self.global_target_pose.pose.position.x, self.global_target_pose.pose.position.y,\
              self.global_target_pose.pose.position.z
        if rospy.is_shutdown():
          return 'preempted'

        return 'succeeded'

    def goal_callback(self, msg):
        print "Received goal:"
        self.global_target_pose = msg
        self.flag = 1
        self.subscriber.unregister()
