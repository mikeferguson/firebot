import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('firebot')
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']
    tree = os.path.join(bringup_dir, 'config', 'behavior_tree.xml')

    return LaunchDescription([

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'nav2_params.yaml')],
            remappings=[('cmd_vel', 'base_controller/command'),
                        ('odom', 'base_controller/odom')]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'nav2_params.yaml')]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'nav2_params.yaml')]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'default_nav_to_pose_bt_xml': tree},
                        os.path.join(bringup_dir, 'config', 'nav2_params.yaml')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])
