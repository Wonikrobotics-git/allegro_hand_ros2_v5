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
    setup_can0_script = os.path.join(allegro_hand_controllers_share, 'scripts', 'setup_can0.sh')

    # Declare launch argument
    declare_visualize_arg = DeclareLaunchArgument(
        'VISUALIZE',
        default_value='false',
        description='Flag to enable/disable visualization'
    )

    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )
    
    declare_type_arg = DeclareLaunchArgument(
        'TYPE',
        default_value='A',
        description='Specify which type to use: A(non-geared) or B(geared)'
    )

    declare_polling_arg = DeclareLaunchArgument(
        'POLLING',
        default_value='true',
        description='true, false for polling the CAN communication'
    )
    
    declare_can_device_arg = DeclareLaunchArgument(
    	'CAN_DEVICE', 
    	default_value='can0',
    	description='Specify CAN port for control multi devices'
    )
    
    declare_num_arg = DeclareLaunchArgument(
    	'NUM',
    	default_value='0',
    	description='Specify AH num for control multi devices'
    )
    
    def setup_can(context):
    	can_port = context.launch_configurations['CAN_DEVICE']
    	commands = [
        	f"sudo ip link set {can_port} down",
        	f"sudo ip link set {can_port} type can bitrate 1000000",
        	f"sudo ip link set {can_port} up"
    	]
    
    	while True:
        	password = getpass.getpass('Enter sudo password: ')
        	success = True
        
        	for cmd in commands:
            		result = os.system(f'echo "{password}" | sudo -S {cmd}')
            		if result != 0:
                		print(f"Command failed: {cmd}")
                		success = False
                		break
        
        	if success:
            		print(f'{can_port} setup completed')
            		break
        	else:
            		print(f'{can_port} setup failed. Please try again.')
    
    	return []  
        
    urdf_path = PythonExpression([
        '"', allegro_hand_controllers_share, '/urdf/allegro_hand_description_', LaunchConfiguration('HAND'),'_', LaunchConfiguration('TYPE'),'.urdf"'
    ])

    return LaunchDescription([
        declare_visualize_arg,
        declare_polling_arg,
        declare_can_device_arg,
        declare_num_arg,
        OpaqueFunction(function=setup_can),
        Node(
            package='allegro_hand_controllers',
            executable='allegro_node_grasp',
            output='screen',
            parameters=[{'hand_info/which_hand': LaunchConfiguration('HAND')}, # Pass HAND argument to parameter
            		{'hand_info/which_type': LaunchConfiguration('TYPE')},
            		{'comm/CAN_CH': LaunchConfiguration('CAN_DEVICE')}],
            arguments=[LaunchConfiguration('POLLING')],
            remappings=[('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
            		('allegroHand/joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"]))
            
            ]
        ),
        Node(
            package='robot_state_publisher',
            output='screen',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            remappings=[
                ('tf', PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tf'"])),
                ('joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
                ('robot_description', 'allegro_hand_description')
            ]
        ),
        # Include the allegro_viz.launch.py file if VISUALIZE is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_controllers_share, 'launch', 'allegro_viz.launch.py')),
            condition=IfCondition(LaunchConfiguration('VISUALIZE')),
            launch_arguments={'NUM': LaunchConfiguration('NUM')}.items()
        )
        
    ])
