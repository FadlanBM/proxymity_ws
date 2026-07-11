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
        DeclareLaunchArgument('rate', default_value='30.0',
                              description='Publish rate in Hz'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Run in simulation mode (no GPIO)'),
        DeclareLaunchArgument('frame_id', default_value='proximity_link',
                              description='Frame ID for Range message'),
        DeclareLaunchArgument('obstacle_threshold_cm', default_value='10.0',
                              description='Obstacle detection threshold in cm for HC-SR04'),
        DeclareLaunchArgument('obstacle_confirm_count', default_value='7',
                              description='Consecutive True raw reads required to confirm an obstacle (L18D80 noise filter)'),

        # HC-SR04 pins
        DeclareLaunchArgument('trigger_pin', default_value='11',
                              description='GPIO line for HC-SR04 TRIG pin'),
        DeclareLaunchArgument('echo_pin', default_value='12',
                              description='GPIO line for HC-SR04 ECHO pin'),

        # L18D80 pin
        DeclareLaunchArgument('out_pin', default_value='144',
                              description='GPIO line for L18D80 OUT pin'),

        # ---------- Controller Launch Arguments ----------
        DeclareLaunchArgument('relative_move_topic', default_value='/relative_move_slow',
                              description='Topic for Vector3 relative move commands'),
        DeclareLaunchArgument('fsm_command_topic', default_value='/fsm_command',
                              description='Topic for FSM integer commands'),
        DeclareLaunchArgument('fsm_command_val', default_value='31',
                              description='FSM command code to send when sensor returns True'),
        DeclareLaunchArgument('initial_relative_y', default_value='-0.5',
                              description='Relative Y move when sensor is clear (strafing)'),
        DeclareLaunchArgument('forward_relative_x', default_value='-0.5',
                              description='Relative X move when sensor detects an obstacle'),
        DeclareLaunchArgument('forward_duration', default_value='3.0',
                              description='Duration in seconds to send forward relative move command'),
        DeclareLaunchArgument('y_move_interval', default_value='0.5',
                              description='Interval in seconds between incremental Y relative move publishes'),
        DeclareLaunchArgument('rotate_relative_z', default_value='180.0',
                              description='Relative Z move to rotate the robot (TODO: confirm value for a true 180-degree turn)'),
        DeclareLaunchArgument('rotate_duration', default_value='2.0',
                              description='Duration in seconds to send the rotation command'),
        DeclareLaunchArgument('flash_area_name', default_value='AREA_2',
                              description='Area name sent to /fsm/area_command to trigger green_light_node detection'),
        DeclareLaunchArgument('flash_command_val', default_value='-1',
                              description='PLACEHOLDER: FSM command to publish once the green flash is confirmed. Change once the real code is known.'),
        DeclareLaunchArgument('confirm_duration', default_value='1.0',
                              description='Duration in seconds sensor must stay True to confirm detection'),
        DeclareLaunchArgument('confirm_nudge_y', default_value='-0.3',
                              description='Relative Y nudge when confirmation fails (nudge back right to re-search)'),

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
                'obstacle_confirm_count': LaunchConfiguration('obstacle_confirm_count'),
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
                'relative_move_topic': LaunchConfiguration('relative_move_topic'),
                'fsm_command_topic': LaunchConfiguration('fsm_command_topic'),
                'publish_rate_hz': 10.0,
                'fsm_command_val': LaunchConfiguration('fsm_command_val'),
                'initial_relative_y': LaunchConfiguration('initial_relative_y'),
                'forward_relative_x': LaunchConfiguration('forward_relative_x'),
                'forward_duration': LaunchConfiguration('forward_duration'),
                'y_move_interval': LaunchConfiguration('y_move_interval'),
                'rotate_relative_z': LaunchConfiguration('rotate_relative_z'),
                'rotate_duration': LaunchConfiguration('rotate_duration'),
                'flash_area_name': LaunchConfiguration('flash_area_name'),
                'flash_command_val': LaunchConfiguration('flash_command_val'),
                'confirm_duration': LaunchConfiguration('confirm_duration'),
                'confirm_nudge_y': LaunchConfiguration('confirm_nudge_y'),
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