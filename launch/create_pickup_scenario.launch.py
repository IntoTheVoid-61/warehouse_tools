from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():

    #--Path to config file--#
    config_file = PathJoinSubstitution(["config","params.yaml"])

    pickup_scenario_node = Node(
        package='warehouse_tools', # package name
        executable='create_pickup_scenario', # executable name (same as in setup.py)
        name='pickup_scenario_node', # name of node
        parameters=[config_file] # params from yaml config file
    )

    #--Launch description--#
    ld = LaunchDescription()

    ld.add_action(pickup_scenario_node)

    return ld