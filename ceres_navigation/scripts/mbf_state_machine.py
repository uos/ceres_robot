#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from move_base_flex_msgs.msg import ExePathAction
from move_base_flex_msgs.msg import GetPathAction
from move_base_flex_msgs.msg import RecoveryAction
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from wait_for_goal import WaitForGoal


def get_path_goal_cb(userdata, goal):
    goal.use_start_pose = False
    goal.target_pose = userdata.target_pose
    goal.waypoints = []
    goal.goal_tolerance = 0.1

def get_path_result_cb(userdata, status, result):
    userdata.path = result.path


def ex_path_goal_cb(userdata, goal):
    goal.path = userdata.path


def ex_path_result_cb(userdata, status, result):
    if result.plugin_msg:
        userdata.error = result.plugin_msg
    elif result.server_msg:
        userdata.error = result.server_msg


def recovery_path_goal_cb(userdata, goal):
    if userdata.clear_costmap_flag == False:
        goal.behavior = 'clear_costmap'
        userdata.clear_costmap_flag = True
    else:
        goal.behavior = 'straf_recovery'
        userdata.clear_costmap_flag = False
    

def main():
    rospy.init_node('mbf_state_machine')

    mbf_sm = smach.StateMachine(outcomes=['preempted'])
    mbf_sm.userdata.path = Path()
    mbf_sm.userdata.previous_state = None
    mbf_sm.userdata.act_pos = None
    mbf_sm.userdata.error = None
    mbf_sm.userdata.error_status = None
    mbf_sm.userdata.goal_position = None
    mbf_sm.userdata.recovery_behavior = None
    mbf_sm.userdata.clear_costmap_flag = False

    with mbf_sm:
        smach.StateMachine.add('WAIT_FOR_GOAL',
                               WaitForGoal(),
                               transitions={'succeeded': 'GET_PATH', 'preempted':'preempted'})
        
        smach.StateMachine.add('GET_PATH',smach_ros.SimpleActionState('move_base_flex/get_path', 
								    GetPathAction,
                                                                    goal_cb = get_path_goal_cb,
								    result_cb = get_path_result_cb,
                                                                    input_keys = ['target_pose'],
                                                                    output_keys = ['path', 'error_msg']),
                                                                    transitions = {'succeeded':'EXE_PATH',
                                                                    'aborted':'WAIT_FOR_GOAL',
                                                                    'preempted':'preempted'},
                                                                    remapping = {'path': 'path'})

        smach.StateMachine.add('EXE_PATH', smach_ros.SimpleActionState('move_base_flex/exe_path', 
								    ExePathAction,
                                                                    goal_cb = ex_path_goal_cb,
                                                                    input_keys = ['path'],
                                                                    output_keys = ['error_msg']),
                                                                    transitions = {'succeeded':'WAIT_FOR_GOAL',
                                                                    'aborted':'RECOVERY',
                                                                    'preempted':'preempted'})
        smach.StateMachine.add('RECOVERY', smach_ros.SimpleActionState('move_base_flex/recovery', 
								    RecoveryAction, 
								    goal_cb = recovery_path_goal_cb,
                                                                    input_keys = ["error", "clear_costmap_flag"],
                                                                    output_keys = ["error_status", 'clear_costmap_flag']),
                                                                    transitions = {'succeeded': 'GET_PATH',
                                                                    'aborted':'WAIT_FOR_GOAL',
                                                                    'preempted':'preempted'})
        
    sis = smach_ros.IntrospectionServer('mbf_state_machine_server', mbf_sm, '/SM_ROOT')
    sis.start()
    outcome = mbf_sm.execute()
    rospy.spin()
    sis.stop()
   
if __name__=="__main__":
    while not rospy.is_shutdown():
        main()


