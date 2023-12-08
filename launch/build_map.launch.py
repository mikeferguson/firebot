from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory('firebot') + '/config/mapper_params_online_sync.yaml'
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
