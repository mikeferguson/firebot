#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # TODO: add URDF with neck position

    # Load the driver config
    driver_config = os.path.join(
        get_package_share_directory('fire_fighter'),
        'config', 'etherbotix.yaml'
    )

    return LaunchDescription([
        # Etherbotix drivers
        Node(
            name='etherbotix',
            package='etherbotix',
            executable='etherbotix_driver',
            parameters=[driver_config],
            remappings=[('odom', 'base_controller/odom')],
            output='screen',
        ),

        # TODO: add FLIR camera
    ])
