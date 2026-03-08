from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_gpio',
            default_value='false',
            description='Use mock GPIO instead of pigpio (for non-Raspberry Pi environments)',
        ),
        Node(
            package='rc_driver',
            executable='rc_driver_node',
            name='twist_to_pwm',
            output='screen',
            parameters=[{
                'use_mock_gpio': LaunchConfiguration('use_mock_gpio'),
            }],
        ),
    ])
