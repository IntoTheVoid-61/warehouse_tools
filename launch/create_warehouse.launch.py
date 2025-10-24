from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #--Path to yaml config file--#
    config = os.path.join(
        get_package_share_directory('warehouse_tools'),
        'config',
        'create_empty_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='warehouse_tools', # package name
            executable='create_empty_warehouse', # executable name (same as in setup.py)
            name='warehouse_creator', # name of node
            output='screen',
            parameters=[config] # params from yaml config file
        )
    ])