#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    drivers_launch = os.path.join(
        get_package_share_directory('firebot'),
        'launch',
        'drivers.launch.py'
    )
    localization_launch = os.path.join(
        get_package_share_directory('firebot'),
        'launch',
        'localization_ndt.launch.py'
    )
    navigation_launch = os.path.join(
        get_package_share_directory('firebot'),
        'launch',
        'navigation.launch.py'
    )

    return LaunchDescription([
        # Main Control Node
        Node(
            name='firebot',
            package='firebot',
            executable='main.py',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([drivers_launch]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([localization_launch]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation_launch]),
        ),
        # Teleop
        Node(
            name='joy',
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 1.0}, ],
        ),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
