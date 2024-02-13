# Copyright (c) 2020, Stratom Inc.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ceres_bringup'),
        'config',
        'sick_tim.yaml'
    )

    # Sick Tim
    sick_tim = Node(
        package='sick_tim',
        executable='sick_tim551_2050001',
        parameters=[config]
    )

    return LaunchDescription([
        sick_tim
    ])
