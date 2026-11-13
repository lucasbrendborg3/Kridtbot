# example_launch.py


from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    listner_node = Node(
        package='linear_actuator_calibration',
        executable='calibration_sequence',
        name='calibration_sequence'
    )

    return LaunchDescription([
        listner_node,
    ])