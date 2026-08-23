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
        DeclareLaunchArgument(
            'pigpiod_host',
            default_value='localhost',
            description='Hostname or IP address of the pigpiod daemon (usually localhost is fine)',
        ),
        DeclareLaunchArgument(
            'pigpiod_port',
            default_value='8888',
            description='Port of the pigpiod daemon (default: 8888)',
        ),
        Node(
            package='rc_driver',
            executable='rc_driver_node',
            name='twist_to_pwm',
            output='screen',
            parameters=[{
                'use_mock_gpio': LaunchConfiguration('use_mock_gpio'),
                'pigpiod_host': LaunchConfiguration('pigpiod_host'),
                'pigpiod_port': LaunchConfiguration('pigpiod_port'),
            }],
        ),
    ])
