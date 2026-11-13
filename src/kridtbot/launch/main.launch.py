"""Brings up the rover (kridtbot, without active suspension), the arm's software
(robot_arm_description/arm_bringup) and the stop-and-go mission node.

Needs both workspaces sourced, the arm's first:
  source ~/robot_arm_ws/install/setup.bash
  source ~/kridtbot_ws/install/setup.bash
  ros2 launch kridtbot main.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    fake_hardware = LaunchConfiguration('fake_hardware')
    detector = LaunchConfiguration('detector')

    kridtbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kridtbot'), 'launch', 'launch_kridtbot.launch.py'])),
        launch_arguments={'use_odrive': 'false'}.items(),
    )

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_arm_description'), 'launch', 'arm_bringup.launch.py'])),
        launch_arguments={'fake_hardware': fake_hardware, 'detector': detector}.items(),
    )

    # The node waits by itself for /diff_cont/odom and /arm/remove_weed, so no start delay is needed.
    mission = Node(
        package='main_weeding',
        executable='mission',
        name='mission',
        output='screen',
        parameters=[{
            'step_distance': ParameterValue(LaunchConfiguration('step_distance'), value_type=float),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'search_timeout': ParameterValue(LaunchConfiguration('search_timeout'), value_type=float),
            'max_stops': ParameterValue(LaunchConfiguration('max_stops'), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('step_distance', default_value='1.0',
                              description='Distance [m] driven between arm cycles'),
        DeclareLaunchArgument('speed', default_value='0.2',
                              description='Driving speed [m/s]; negative drives backwards'),
        DeclareLaunchArgument('search_timeout', default_value='0.0',
                              description="Arm search timeout [s] per stop; 0 = the arm's default (20 s)"),
        DeclareLaunchArgument('max_stops', default_value='0',
                              description='Number of arm cycles before the mission ends; 0 = unlimited'),
        DeclareLaunchArgument('fake_hardware', default_value='false',
                              description='Passed to arm_bringup: fake arm instead of camera + Pi'),
        DeclareLaunchArgument('detector', default_value='hsv', choices=['hsv', 'yolo'],
                              description='Passed to arm_bringup: perception chain'),
        kridtbot,
        arm,
        mission,
    ])
