from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='potential_fields',
            executable='differential_odometry',
            name='differential_odometry',
            output='screen'
        ),
        Node(
            package='potential_fields',
            executable='waypoint_sequencer',
            name='waypoint_sequencer',
            output='screen'
        ),
        Node(
            package='potential_fields',
            executable='vpf_controller',
            name='vpf_controller',
            output='screen'
        ),
    ])
