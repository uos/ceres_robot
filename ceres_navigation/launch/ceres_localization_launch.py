from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import HasNodeParams, RewrittenYaml
import launch_ros.actions




def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false'
        )
    


    # start_map_server = GroupAction(
    #         actions=[
    #             SetParameter('use_sim_time', is_sim),
    #             Node(
    #                 package='nav2_map_server',
    #                 executable='map_saver_server',
    #                 output='screen',
    #                 respawn_delay=2.0,
    #                 arguments=['--ros-args', '--log-level', log_level],
    #                 parameters=[configured_params]),
    #             Node(
    #                 package='nav2_lifecycle_manager',
    #                 executable='lifecycle_manager',
    #                 name='lifecycle_manager_slam',
    #                 output='screen',
    #                 arguments=['--ros-args', '--log-level', log_level],
    #                 parameters=[{'autostart': autostart},
    #                             {'node_names': lifecycle_nodes}])
    #         ])

    return LaunchDescription([
        is_sim_arg,
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{"use_sim_time" : is_sim},os.path.join(get_package_share_directory("ceres_localization"), 'cfg', 'ekf.yaml')],
           ),
])