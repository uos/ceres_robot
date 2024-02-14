"""Launch the velodyne driver node in a composable container with default configuration."""

import os

import ament_index_python.packages

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
import sys
import yaml


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ceres_bringup'),
        'config')
    param_config = os.path.join(config_directory, 'VLP16-velodyne-params.yaml')
    param_config_pcd = os.path.join(config_directory, 'VLP16-trans-params.yaml')
    param_config_laser = os.path.join(config_directory, 'VLP16-velodyne_laserscan_node-params.yaml')

    share_dir_velo_pcd = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')

    

    with open(param_config, 'r') as f:
        params_velo_driver = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
    with open(param_config_pcd, 'r') as f:
        params_velo_pcd = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
        params_velo_pcd['calibration'] = os.path.join(share_dir_velo_pcd, 'params', 'VLP16db.yaml')
    with open(param_config_laser, 'r') as f:
        params_velo_laser = yaml.safe_load(f)['velodyne_laserscan_node']['ros__parameters']
    container = ComposableNodeContainer(
            name='velodyne_driver_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[params_velo_driver]),
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[params_velo_pcd],
                    remappings=[('/velodyne_points', '/velodyne/points')]),
                ComposableNode(
                    package='velodyne_laserscan',
                    plugin='velodyne_laserscan::VelodyneLaserScan',
                    name='velodyne_laserscan_node',
                    parameters=[params_velo_laser],
                    remappings=[('/scan', '/velodyne/scan'), ('/velodyne_points', '/velodyne/points')]),
            ],
            output='both',
    )

    laser2pcd = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/velodyne/points']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/velodyne/multi_scan'])],
            parameters=[{
                'target_frame' : 'base_footprint',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    return LaunchDescription([container, laser2pcd])
