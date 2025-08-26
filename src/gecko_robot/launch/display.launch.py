import os 
import launch_ros
import launch.conditions
from launch.substitutions import Command, LaunchConfiguration
import xacro 
from ament_index_python.packages import get_package_share_directory

packageName = 'gecko_robot'

# xacroRelativePath = 'model/onshape_robot.xacro'
xacroRelativePath = 'model/gecko_model.xacro'

rvizRelativePath = 'config/config.rviz'

def generate_launch_description():

    share_dir = get_package_share_directory(packageName) 
    xacroFile = os.path.join(share_dir, 'model', 'gecko_model.xacro')
    robotDescription = xacro.process_file(xacroFile).toxml()

    rvizPath = os.path.join(get_package_share_directory(packageName), rvizRelativePath)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                    'use_sim_time': False}]
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output="screen",
        arguments=['-d', rvizPath],
        parameters=[{"use_sim_time": False}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='flag to enable joint state pub GUI'),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])