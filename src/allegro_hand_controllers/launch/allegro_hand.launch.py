from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
import getpass

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')
    allegro_hand_moveit_share = get_package_share_directory('allegro_hand_moveit')
    setup_can0_script = os.path.join(allegro_hand_controllers_share, 'scripts', 'setup_can0.sh')

    # Declare launch argument
    declare_visualize_arg = DeclareLaunchArgument(
        'VISUALIZE',
        default_value='false',
        description='Flag to enable/disable visualization'
    )
    
    declare_moveit_arg = DeclareLaunchArgument(
        'MOVEIT',
        default_value='false',
        description='Flag to enable/disable visualization'
    )

    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )

    declare_polling_arg = DeclareLaunchArgument(
        'POLLING',
        default_value='true',
        description='true, false for polling the CAN communication'
    )
    


    def setup_can0(context):
        while True:
            password = getpass.getpass('Enter sudo password: ')
            result = os.system(f'echo "{password}" | sudo -S bash -c ". {setup_can0_script}"')
            if result == 0:
                print('can0 setup completed')
                break
            else:
                print('can0 setup failed. Please try again.')
        return []
        
    urdf_path = PythonExpression([
        '"', allegro_hand_controllers_share, '/urdf/allegro_hand_description_', LaunchConfiguration('HAND'), '.urdf"'
    ])

    return LaunchDescription([
        declare_visualize_arg,
        declare_polling_arg,
        OpaqueFunction(function=setup_can0),
        Node(
            package='allegro_hand_controllers',
            executable='allegro_node_grasp',
            output='screen',
            parameters=[{'hand_info/which_hand': LaunchConfiguration('HAND')}], # Pass HAND argument to parameter
            arguments=[LaunchConfiguration('POLLING')]
        ),
        Node(
            package='robot_state_publisher',
            output='screen',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            remappings=[
                ('tf', 'allegroHand/tf'),
                ('joint_states', 'allegroHand/joint_states'),
                ('robot_description', 'allegro_hand_description')
            ]
        ),
        # Include the allegro_viz.launch.py file if VISUALIZE is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_controllers_share, 'launch', 'allegro_viz.launch.py')),
            condition=IfCondition(LaunchConfiguration('VISUALIZE'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_moveit_share, 'launch', 'demo.launch.py')),
            condition=IfCondition(LaunchConfiguration('MOVEIT'))    
        )
    ])
