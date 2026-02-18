from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="eno2"),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),

        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_controlled': arm_controlled,  # string ('left'|'right'|'both')
                'enable_arm_ui': ParameterValue(enable_arm_ui, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),
    ])
