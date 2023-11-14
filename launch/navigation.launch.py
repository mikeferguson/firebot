import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('fire_fighter')
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']

    #default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'default_nav_to_pose_bt_xml',
            default_value=os.path.join(bringup_dir, 'behavior_trees', 'default.xml'),
            description='Full path to the behavior tree xml file to use'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'nav2_params.yaml')],
            remappings=[('cmd_vel', 'base_controller/command'), ]),

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
            parameters=[{'default_nav_to_pose_bt_xml': os.path.join(bringup_dir, 'config', 'behavior_tree.xml')},
                        os.path.join(bringup_dir, 'config', 'nav2_params.yaml')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])

