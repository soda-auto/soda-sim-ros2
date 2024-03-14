from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    max_speed = LaunchConfiguration('max_speed')
    max_angle = LaunchConfiguration('max_angle')

    declare_max_speed_cmd = DeclareLaunchArgument(
        'max_speed',
        default_value='25.0',
        description='Maximum speed.'
    )

    declare_max_angle_cmd = DeclareLaunchArgument(
        'max_angle',
        default_value='0.5',
        description='Maximum steering angle.'
    )

    ackermann_drive_joyop_cmd = Node(
        package='ackermann_drive_teleop',
        executable='joyop.py',
        name='ackermann_drive_joyop',
        output='screen',
        arguments=[max_speed, max_angle],
    )

    joy_node_cmd = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen'
    )

    return LaunchDescription([
        declare_max_speed_cmd,
        declare_max_angle_cmd,
        ackermann_drive_joyop_cmd,
        joy_node_cmd
    ])
