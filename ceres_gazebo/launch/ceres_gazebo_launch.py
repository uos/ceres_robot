from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from scripts import GazeboRosPaths
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('ceres_gazebo'), 'config')

    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    # Needed for gazebo to find models
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    print("GAZEBO_MODEL_PATH", model_path)
    print("GAZEBO_PLUGIN_PATH", plugin_path)
    print("GAZEBO_RESOURCE_PATH", media_path)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ceres_description"), "urdf", "ceres.urdf.xacro"]
            ),
            " ",
            "name:=ceres",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            # "gazebo_controllers:=",
            # config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True, "publish_frequency": 100.0}, robot_description],
    )

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(get_package_share_directory('ceres_gazebo'), 'worlds', 'avz_collada.world'),
                'verbose': 'true',
            }.items()
        )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ceres',
        arguments=['-entity',
                   'ceres',
                   '-topic',
                   'robot_description'],
        output='screen',
    )


    ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ceres_localization'), 'launch', 'ceres_localization_launch2.py')
            ),
            launch_arguments={
                'is_sim': 'true',
            }.items()
        )
    
    imufilter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[os.path.join(config_dir, 'madgwick.yaml')],
        )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gzserver)
    ld.add_action(spawn_robot)
    ld.add_action(imufilter)
    ld.add_action(ekf)

    return ld
