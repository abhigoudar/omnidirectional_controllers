from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description = LaunchDescription()

    control_config_arg = DeclareLaunchArgument(
        name="controller_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("omnidirectional_controllers"), "config/position_controller_config.yaml"
        ])
    )
    #
    #
    description.add_action(control_config_arg)

    description.add_action(
        Node(
            executable="omni_pos_controller",
            name="position_controller",
            package="omnidirectional_controllers",
            arguments=[LaunchConfiguration("controller_config")],
            output="screen",
            emulate_tty="True"
        )
    )

    return description