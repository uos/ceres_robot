import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("ceres_localization"),'config', 'gmapping.yaml'),
        description='Full path to the ROS2 parameters file to use for the gmapping node')

    # what does this do?
    map_saver = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ])
    
    life = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_saver_server_life',
            output='screen',
            arguments=['--ros-args', '--log-level', "INFO"],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_saver_server']}])
    
    print("SLAM")
    slam = Node(
            package='slam_gmapping', 
            executable='slam_gmapping_node',
            name='slam_gmapping',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                slam_params_file
            ]
        )
    

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(slam)
    
    ld.add_action(map_saver)
    ld.add_action(life)
    


    return ld
