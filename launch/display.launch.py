import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    # Path to the package.
    package_path = launch_ros.substitutions.FindPackageShare(package='robko01_ros2').find('robko01_ros2')

    # Path to the URDF model.
    urdf_model       = os.path.join(package_path, 'urdf/robko01.urdf')

    # Path to the RViz configuration.
    rviz_config_file = os.path.join(package_path, 'rviz', 'default.rviz')

    # Load the URFT model.
    robot_desc = b""
    with open(urdf_model,'r') as infp:
        robot_desc = infp.read()
    
    # Parameters (robot model)
    params = {
        'robot_description': robot_desc
    }
    
    #
    robot_state_publisher_node =launch_ros.actions.Node(
    package='robot_state_publisher',
	executable='robot_state_publisher',
    output='screen',
    parameters=[params])
    
    #
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    #
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )


    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='This is a flag for joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_model,
                                            description='Path to the urdf model file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 