from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false'
        )
    
    return LaunchDescription([
        is_sim_arg,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{"use_sim_time" : is_sim},os.path.join(get_package_share_directory("ceres_localization"), 'config', 'ekf.yaml')],
           ),
        ])