from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution

def generate_launch_description():
    message_display_launch_arg = DeclareLaunchArgument(
        'message_display',
        default_value = TextSubstitution(text='screen'),
        description = 'message_display'
    )
    serial_port_launch_arg = DeclareLaunchArgument(
        'serial_port',
        default_value = TextSubstitution(text='/dev/recipe.driver.battery'),
        description = 'serial_port'
    )
    baud_rate_launch_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value = TextSubstitution(text='9600'),
        description = 'baud_rate'
    )
    bms_model_launch_arg = DeclareLaunchArgument(
        'bms_model',
        default_value = TextSubstitution(text='qucc'),
        description = 'bms_model'
    )
    QUERY_BATTERY_SEC_launch_arg = DeclareLaunchArgument(
        'QUERY_BATTERY_SEC',
        default_value = TextSubstitution(text='0.5'),
        description = 'QUERY_BATTERY_SEC'
    )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('battery'),
                'launch',
                'base.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        message_display_launch_arg,
        serial_port_launch_arg,
        baud_rate_launch_arg,
        bms_model_launch_arg,
        QUERY_BATTERY_SEC_launch_arg,
        launch,
    ])