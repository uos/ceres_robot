from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument



def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false'
        )
    
    logger = launch.substitutions.LaunchConfiguration("log_level")
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
                "log_level",
                default_value=["debug"],
                description="Logging level",
        ),
        is_sim_arg,
        launch_ros.actions.Node(
            package='robot_pose_ekf',
            executable='robot_pose_ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('imu_data', '/imu/data'),
            ],
            parameters=[{"use_sim_time" : is_sim},os.path.join(get_package_share_directory("ceres_localization"), 'cfg', 'robot_pose_ekf.yaml')],
            arguments=['--ros-args', '--log-level', logger]
           ),
])