from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Sensor type
        DeclareLaunchArgument('sensor_type', default_value='l18d80',
                              description='Sensor type: "hcsr04" or "l18d80"'),

        # Common
        DeclareLaunchArgument('gpio_chip', default_value='/dev/gpiochip0',
                              description='GPIO chip device path'),
        DeclareLaunchArgument('rate', default_value='10.0',
                              description='Publish rate in Hz'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Run in simulation mode (no GPIO)'),
        DeclareLaunchArgument('frame_id', default_value='proximity_link',
                              description='Frame ID for Range message'),
        DeclareLaunchArgument('obstacle_threshold_cm', default_value='10.0',
                              description='Obstacle detection threshold in cm for HC-SR04'),
        DeclareLaunchArgument('obstacle_on_delay', default_value='0.0',
                              description='Delay for False -> True (obstacle detected) transition in seconds'),
        DeclareLaunchArgument('obstacle_off_delay', default_value='0.0',
                              description='Delay for True -> False (obstacle cleared) transition in seconds'),
        DeclareLaunchArgument('oscillation_window', default_value='1.0',
                              description='Time window to count toggles for oscillation detection'),
        DeclareLaunchArgument('oscillation_max_toggles', default_value='3',
                              description='Max toggles in window before suppressing to False'),

        # HC-SR04 pins
        DeclareLaunchArgument('trigger_pin', default_value='11',
                              description='GPIO line for HC-SR04 TRIG pin'),
        DeclareLaunchArgument('echo_pin', default_value='12',
                              description='GPIO line for HC-SR04 ECHO pin'),

        # L18D80 pin
        DeclareLaunchArgument('out_pin', default_value='144',
                              description='GPIO line for L18D80 OUT pin'),

        # ---------- Controller Launch Arguments ----------
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Topic for Twist velocity commands'),
        DeclareLaunchArgument('fsm_command_topic', default_value='/fsm_command',
                              description='Topic for FSM integer commands'),
        DeclareLaunchArgument('fsm_command_val', default_value='31',
                              description='FSM command code to send when sensor returns True'),
        DeclareLaunchArgument('initial_linear_y', default_value='-0.05',
                              description='Linear y speed when moving initially (positive for strafing left)'),
        DeclareLaunchArgument('forward_linear_x', default_value='0.02',
                              description='Linear x speed when moving forward'),
        DeclareLaunchArgument('forward_duration', default_value='1.5',
                              description='Duration in seconds to send forward command'),

        # Proximity Sensor Node
        Node(
            package='proxymity',
            executable='proxymity_node',
            name='proximity_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'sensor_type': LaunchConfiguration('sensor_type'),
                'gpio_chip': LaunchConfiguration('gpio_chip'),
                'publish_rate_hz': LaunchConfiguration('rate'),
                'simulate': LaunchConfiguration('simulate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'obstacle_threshold_cm': LaunchConfiguration('obstacle_threshold_cm'),
                'obstacle_on_delay': LaunchConfiguration('obstacle_on_delay'),
                'obstacle_off_delay': LaunchConfiguration('obstacle_off_delay'),
                'oscillation_window': LaunchConfiguration('oscillation_window'),
                'oscillation_max_toggles': LaunchConfiguration('oscillation_max_toggles'),
                'trigger_pin': LaunchConfiguration('trigger_pin'),
                'echo_pin': LaunchConfiguration('echo_pin'),
                'out_pin': LaunchConfiguration('out_pin'),
            }],
        ),

        # Proximity Controller Node
        Node(
            package='proxymity',
            executable='proxymity_controller_node',
            name='proximity_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'sensor_topic': '/proximity/obstacle',
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'fsm_command_topic': LaunchConfiguration('fsm_command_topic'),
                'publish_rate_hz': 10.0,
                'fsm_command_val': LaunchConfiguration('fsm_command_val'),
                'initial_linear_y': LaunchConfiguration('initial_linear_y'),
                'forward_linear_x': LaunchConfiguration('forward_linear_x'),
                'forward_duration': LaunchConfiguration('forward_duration'),
                'right_linear_y': LaunchConfiguration('initial_linear_y'),
                'backward_linear_x': LaunchConfiguration('forward_linear_x'),
                'backward_duration': LaunchConfiguration('forward_duration'),
            }],
        ),

        # Include Green Light Detection launch file (use_realsense:=true by default)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('proxymity'), 'launch', 'green_light_launch.py']
                )
            ),
            launch_arguments={
                'use_realsense': 'true',
            }.items()
        ),
    ])
