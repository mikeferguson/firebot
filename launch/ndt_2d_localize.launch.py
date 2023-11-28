#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('firebot')

    return LaunchDescription([

        Node(
            package='ndt_2d',
            executable='ndt_2d_map_node',
            name='ndt_2d_map_node',
            output='screen',
            parameters=[{'resolution': 0.01,
                         'ndt_resolution': 0.05,
                         'occupancy_threshold': 0.05,
                         'enable_mapping': False,
                         'map_file': os.path.join(bringup_dir, 'config', 'full_map')}]),
    ])
