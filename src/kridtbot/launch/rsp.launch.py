import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # sim time argument
    use_sim = LaunchConfiguration('use_sim')
    active_suspension = LaunchConfiguration('active_suspension')
    use_odrive = LaunchConfiguration('use_odrive')
    mock_left_wheels = LaunchConfiguration('mock_left_wheels')
    mock_right_wheels = LaunchConfiguration('mock_right_wheels')

    # Process URDF
    pkg_path = os.path.join(get_package_share_directory('kridtbot'))

    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro') # main xacro

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    # value_type=str: pass the URDF as plain text, so ROS doesn't try to read it as YAML
    # (a ': ' anywhere in it, e.g. in a comment, made that fail)
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim_system:=', use_sim, ' sim_mode:=', use_sim, ' active_suspension:=', active_suspension, ' use_odrive:=', use_odrive,
                 ' mock_left_wheels:=', mock_left_wheels, ' mock_right_wheels:=', mock_right_wheels]),
        value_type=str)
    
    # robot_state_publisher node on topic "robot_description"
    params_robot_state_publisher = {'robot_description': robot_description_config, 'use_sim_time': use_sim}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_robot_state_publisher]
    )

    params_joint_state_publisher = {'robot_description': robot_description_config}
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params_joint_state_publisher]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='uses ros2_control if true'),
        DeclareLaunchArgument(
            'active_suspension',
            default_value='false',
            description='Active (IMU) suspension; the suspension joints are prismatic if this or use_odrive is true, fixed otherwise'),
        DeclareLaunchArgument(
            'use_odrive',
            default_value='false',
            description='Include the ODrive (suspension actuator) hardware interface on can_odrive if true'),
        DeclareLaunchArgument(
            'mock_left_wheels',
            default_value='false',
            description='Simulate the left wheels instead of driving them over can_l (for testing with one adapter)'),
        DeclareLaunchArgument(
            'mock_right_wheels',
            default_value='false',
            description='Simulate the right wheels instead of driving them over can_r (for testing with one adapter)'),
        node_robot_state_publisher,
        #rviz2_node,
    ])
