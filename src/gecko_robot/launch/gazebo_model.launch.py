
import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description(): 

    packageName = 'gecko_robot'

    share_dir = get_package_share_directory(packageName) 

    share_parent = os.path.dirname(share_dir)

    # Append that parent to GZ_SIM_RESOURCE_PATH so gz can resolve model://gecko_robot/...
    add_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=share_parent
    )

    xacroFile = os.path.join(share_dir, 'model', 'gecko_model.xacro')
    robotDescription = xacro.process_file(xacroFile).toxml()
    
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 
                                                                        'launch', 'gz_sim.launch.py'))
    
    # gz args string instead of list?
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
                                            launch_arguments={'gz_args': '-r -v -v4 empty.sdf',
                                                            'on_exit_shutdown': 'true'}.items())
    
    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', packageName,
            '-topic', 'robot_description',
            '-z', '4.70'

        ],
        output='screen'
    )

    # Robot State Published Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                    'use_sim_time': True}]
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

    launchDescriptioObject = LaunchDescription()

    launchDescriptioObject.add_action(add_gz_resource_path)
    launchDescriptioObject.add_action(gazeboLaunch)
    launchDescriptioObject.add_action(spawnModelNodeGazebo)
    launchDescriptioObject.add_action(nodeRobotStatePublisher)    
    launchDescriptioObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptioObject