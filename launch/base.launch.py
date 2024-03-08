from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch.actions import OpaqueFunction

def evaluate_node(context, *args, **kwargs):
    outputs = LaunchConfiguration('message_display').perform(context)
    node = Node(
        package = 'battery',
        executable = 'battery',
        name = 'battery',
        output = {
            'stdout': outputs,
            'stderr': outputs,
        },
        emulate_tty = True,
        parameters = [{
            "message_display": LaunchConfiguration('message_display'),
            "serial_port": LaunchConfiguration('serial_port'),
            "baud_rate": LaunchConfiguration('baud_rate'),
            "slave_num": LaunchConfiguration('slave_num'),
            "node_name": LaunchConfiguration('node_name'),
            "bms_model": LaunchConfiguration('bms_model'),
            "QUERY_BATTERY_SEC": LaunchConfiguration('QUERY_BATTERY_SEC'),
        }]
    )

    return [node]

def generate_launch_description():
    message_display_launch_arg = DeclareLaunchArgument(
        "message_display",
        default_value = TextSubstitution(text="screen"),
        description = "message_display"
    )
    serial_port_launch_arg = DeclareLaunchArgument(
        "serial_port",
        default_value = TextSubstitution(text="/dev/recipe.driver.battery"),
        description = "serial_port"
    )
    baud_rate_launch_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value = TextSubstitution(text="19200"),
        description = "baud_rate"
    )
    slave_num_launch_arg = DeclareLaunchArgument(
        "slave_num",
        default_value = TextSubstitution(text="10"),
        description = "slave_num"
    )
    node_name_launch_arg = DeclareLaunchArgument(
        "node_name",
        default_value = TextSubstitution(text="battery"),
        description = "node_name"
    )
    bms_model_launch_arg = DeclareLaunchArgument(
        "bms_model",
        default_value = TextSubstitution(text="unknown"),
        description = "bms_model"
    )
    QUERY_BATTERY_SEC_launch_arg = DeclareLaunchArgument(
        "QUERY_BATTERY_SEC",
        default_value = TextSubstitution(text="1.0"),
        description = "QUERY_BATTERY_SEC"
    )

    return LaunchDescription([
        message_display_launch_arg,
        serial_port_launch_arg,
        baud_rate_launch_arg,
        slave_num_launch_arg,
        node_name_launch_arg,
        bms_model_launch_arg,
        QUERY_BATTERY_SEC_launch_arg,
        OpaqueFunction(function=evaluate_node),
    ])