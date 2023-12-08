#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # TODO: add mux between nav and joystick
        Node(
            name='joy',
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 5.0}, ],
        ),

        # Teleop
        Node(
            name='teleop',
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[{'enable_button': 4,
                         'axis_linear.x': 4,
                         'scale_linear.x': 1.0,
                         'axis_angular.yaw': 0,
                         'scale_angular.yaw': 3.0}],
            remappings=[('cmd_vel', 'base_controller/command')],
            output='screen',
        ),

    ])
