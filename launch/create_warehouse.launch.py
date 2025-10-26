from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    #--Path to config file--#
    config_file = PathJoinSubstitution(["config","params.yaml"])

    create_warehouse_node = Node(
        package='warehouse_tools', # package name
        executable='create_empty_warehouse', # executable name (same as in setup.py)
        name='warehouse_creator_node', # name of node
        parameters=[config_file] # params from yaml config file
    )

    #--Launch description--#
    ld = LaunchDescription()

    ld.add_action(create_warehouse_node)

    return ld

