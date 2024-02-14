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
    param_config_pcd2scan = os.path.join(config_directory, 'pcd2scan.yaml')

    share_dir_velo_pcd = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')

    

    with open(param_config, 'r') as f:
        params_velo_driver = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
    with open(param_config_pcd, 'r') as f:
        params_velo_pcd = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
        params_velo_pcd['calibration'] = os.path.join(share_dir_velo_pcd, 'params', 'VLP16db.yaml')
    with open(param_config_laser, 'r') as f:
        params_velo_laser = yaml.safe_load(f)['velodyne_laserscan_node']['ros__parameters']
    with open(param_config_pcd2scan, 'r') as f:
         params_velo_pcs2scan = yaml.safe_load(f)['pcd2scan']['ros__parameters']

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
            remappings=[('cloud_in', '/velodyne/points'),
                        ('scan', '/velodyne/multi_scan')],
            parameters=[params_velo_pcs2scan],
            name='pointcloud_to_laserscan'
        )

    return LaunchDescription([container, laser2pcd])
