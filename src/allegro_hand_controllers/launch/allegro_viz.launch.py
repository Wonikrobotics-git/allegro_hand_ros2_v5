import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')

    rviz_config_file = os.path.join(allegro_hand_controllers_share, 'urdf', 'allegro_hand_config.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            remappings=[
                ('tf', 'allegroHand/tf')
            ]
        )
    ])
