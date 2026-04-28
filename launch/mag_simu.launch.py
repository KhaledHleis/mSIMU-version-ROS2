from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

Configuration_file = 'Test.yaml'

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('simulation_ros2'),
        'config',
        Configuration_file
    )

    return LaunchDescription([
        Node(
            package='simulation_ros2',
            executable='MagSIMU',
            name='mag_simu_node',
            output='screen',
            parameters=[config]
        )
    ])