#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # TODO: add URDF with neck position
    bringup_dir = get_package_share_directory('fire_fighter')
    urdf_path = os.path.join(bringup_dir, 'config', 'fire_fighter.urdf')

    # Load the URDF into a parameter
    urdf = open(urdf_path).read()

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

        # URDF and TF publishing
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf,
                         'publish_frequency': 100.0}],
        ),

        # FLIR Lepton driver
        Node(
            name='lepton_driver',
            package='fire_fighter',
            executable='lepton_driver.py',
            remappings=[('image', 'lepton/image'),
                        ('region', 'lepton/flame_region')],
        ),

        # LD06 lidar publisher
        Node(
            name='ld06_publisher',
            package='etherbotix',
            executable='ld06_publisher_node',
        ),
    ])
