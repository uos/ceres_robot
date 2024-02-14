from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='false'
        )
    
    logger = LaunchConfiguration("log_level")
    
    return LaunchDescription([
        DeclareLaunchArgument(
                "log_level",
                default_value=["debug"],
                description="Logging level",
        ),
        is_sim_arg,
        Node(
            package='robot_pose_ekf',
            executable='robot_pose_ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('imu_data', '/imu/data'),
            ],
            parameters=[{"use_sim_time" : is_sim},os.path.join(get_package_share_directory("ceres_localization"), 'config', 'robot_pose_ekf.yaml')],
            arguments=['--ros-args', '--log-level', logger]
           ),
])