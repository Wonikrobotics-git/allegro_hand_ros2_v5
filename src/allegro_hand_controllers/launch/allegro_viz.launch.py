import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')
    
    declare_num_arg = DeclareLaunchArgument('NUM', default_value='0')

    rviz_config_file = os.path.join(allegro_hand_controllers_share, 'urdf', 'allegro_hand_config.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            remappings=[
                ('tf', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/tf'"]))
            ]
        )
    ])
