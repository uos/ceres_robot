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

def generate_launch_description():

    # Launch args
    prefix = LaunchConfiguration('prefix')

    config_dir = os.path.join(get_package_share_directory('ceres_bringup'), 'config')

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
        parameters=[{
            'use_sim_time': True, 
            "publish_frequency": 100.0}, robot_description],
    )

    ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ceres_localization'), 'launch', 'ceres_localization_launch2.py')
            ),
            launch_arguments={
                'is_sim': 'false',
                'num_wheels' : "4",
                'wheel_radius' :"0.135",
                'axis_length' : "0.435"
            }.items()
        )
    
    volksbot_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('volksbot_driver'), 'launch', 'volksbot.py')
            ),
            launch_arguments={
                'is_sim': 'false',
            }.items()
        )
    
    phidgets_imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('phidgets_spatial'), 'launch', 'spatial-launch.py')
            ),
            launch_arguments={
                'is_sim': 'false',
            }.items()
        )
    
    vlp16 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ceres_bringup'), 'launch', 'velodyne_launch.py')
            ),
        )

    sick_tim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ceres_bringup'), 'launch', 'sick_tim_launch.py')
            ),
        )
    
    cam_nexigo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ceres_bringup'), 'launch', 'nexigo_launch.py')
            ),
        )
    
    
    imufilter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[os.path.join(config_dir, 'madgwick.yaml')],
        )
    
    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(imufilter)
    ld.add_action(volksbot_driver)
    ld.add_action(ekf)
    ld.add_action(phidgets_imu)
    ld.add_action(sick_tim)
    ld.add_action(vlp16)
    # ld.add_action(cam_nexigo)

    return ld
