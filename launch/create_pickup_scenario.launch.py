from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('warehouse_tools'),
        'config',
        'create_pickup_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='warehouse_tools',
            executable='create_pickup_scenario',
            name='pickup_scenario_node',
            output='screen',
            parameters=[config]
        )
    ])