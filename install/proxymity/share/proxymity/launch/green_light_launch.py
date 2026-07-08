from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port connected to the Teensy',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate for the Teensy connection',
    )
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='2',
        description='OpenCV camera index for green light detection',
    )
    csi_sensor_id_arg = DeclareLaunchArgument(
        'csi_sensor_id',
        default_value='0',
        description='CSI sensor port on Jetson Orin (0 or 1)',
    )
    use_csi_arg = DeclareLaunchArgument(
        'use_csi',
        default_value='false',
        description='Whether to use CSI camera on Jetson Orin (set to false for USB/RealSense cameras)',
    )
    use_realsense_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='true',
        description='Whether to use RealSense ROS 2 topics instead of OpenCV VideoCapture',
    )
    rs_serial_arg = DeclareLaunchArgument(
        'rs_serial',
        default_value='918512073244',
        description='Serial number of the RealSense camera to use',
    )
    launch_realsense_arg = DeclareLaunchArgument(
        'launch_realsense',
        default_value='true',
        description='Whether to automatically launch the realsense2_camera driver node',
    )
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/camera/color/image_raw',
        description='RealSense color image topic to subscribe to',
    )
    show_debug_window_arg = DeclareLaunchArgument(
        'show_debug_window',
        default_value='true',
        description='Whether to show the OpenCV debug windows',
    )

    # Realsense camera driver launch configuration
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )
        ),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_realsense'), "' == 'true' and '",
                LaunchConfiguration('launch_realsense'), "' == 'true'"
            ])
        ),
        launch_arguments={
            'serial_no': PythonExpression(["'\"' + str(", LaunchConfiguration('rs_serial'), ") + '\"'"]),
            'align_depth.enable': 'true',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
        }.items()
    )

    node = Node(
        package='proxymity',
        executable='green_light_node',
        name='green_light_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'camera_index': LaunchConfiguration('camera_index'),
                'csi_sensor_id': LaunchConfiguration('csi_sensor_id'),
                'use_csi': LaunchConfiguration('use_csi'),
                'use_realsense': LaunchConfiguration('use_realsense'),
                'color_topic': LaunchConfiguration('color_topic'),
                'show_debug_window': LaunchConfiguration('show_debug_window'),
            }
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            baud_rate_arg,
            camera_index_arg,
            csi_sensor_id_arg,
            use_csi_arg,
            use_realsense_arg,
            rs_serial_arg,
            launch_realsense_arg,
            color_topic_arg,
            show_debug_window_arg,
            realsense_launch,
            node,
        ]
    )
