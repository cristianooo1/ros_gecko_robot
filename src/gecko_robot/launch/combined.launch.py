import os 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description(): 

    packageName = 'gecko_robot'
    rvizRelativePath = 'config/config.rviz' 

    share_dir = get_package_share_directory(packageName) 

    share_parent = os.path.dirname(share_dir)

    add_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=share_parent
    )

    xacroFile = os.path.join(share_dir, 'model', 'gecko_model.xacro')
    robotDescription = xacro.process_file(xacroFile).toxml()

    rvizPath = os.path.join(get_package_share_directory(packageName), rvizRelativePath) 

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    # world_file = LaunchConfiguration('world_file')

    world_path = os.path.join(share_dir, 'meshes', 'empty.world')
    

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 
                                                                        'launch', 
                                                                        'gz_sim.launch.py'))
    
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
                                            launch_arguments={'gz_args': f'-r -v -v4 {world_path}',
                                                            'on_exit_shutdown': 'true'}.items())

    # gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
    #                                         launch_arguments={'gz_args': ['-r -v -v4', world_path],
    #                                                         'on_exit_shutdown': 'true'}.items())
    
    
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', packageName,
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z
        ],
        output='screen'
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                    'use_sim_time': True}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output="screen",
        arguments=['-d', rvizPath],
        parameters=[{"use_sim_time": True}]
    )

    bridge_params = os.path.join(share_dir,
                                'parameters',
                                'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='X coordinate to spawn the robot at'
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Y coordinate to spawn the robot at'
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='4.25',
            description='Z coordinate to spawn the robot at (above the ground - min 0.25)'
        ),
        add_gz_resource_path,
        gazeboLaunch,
        spawnModelNodeGazebo,
        nodeRobotStatePublisher,
        joint_state_publisher_gui_node,
        rviz_node,
        start_gazebo_ros_bridge_cmd
        
    ])