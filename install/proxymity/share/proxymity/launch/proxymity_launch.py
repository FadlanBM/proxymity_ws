from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trigger_pin', default_value='11',
                              description='GPIO line for TRIG pin'),
        DeclareLaunchArgument('echo_pin', default_value='12',
                              description='GPIO line for ECHO pin'),
        DeclareLaunchArgument('gpio_chip', default_value='/dev/gpiochip0',
                              description='GPIO chip device path'),
        DeclareLaunchArgument('rate', default_value='10.0',
                              description='Publish rate in Hz'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Run in simulation mode (no GPIO)'),
        DeclareLaunchArgument('frame_id', default_value='proximity_link',
                              description='Frame ID for Range message'),

        Node(
            package='proxymity',
            executable='proxymity_node',
            name='proximity_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'trigger_pin': LaunchConfiguration('trigger_pin'),
                'echo_pin': LaunchConfiguration('echo_pin'),
                'gpio_chip': LaunchConfiguration('gpio_chip'),
                'publish_rate_hz': LaunchConfiguration('rate'),
                'simulate': LaunchConfiguration('simulate'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
