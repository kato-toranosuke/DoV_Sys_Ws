from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value=EnvironmentVariable('ROBOT_ID'),
            description='prefix for node name'
        ),
        Node(
            package='robot',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='robot_main',
            name=['robot', LaunchConfiguration('robot_id'), '_robot_main']
        ),
        Node(
            package='robot',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='control_led',
            name=['robot', LaunchConfiguration('robot_id'), '_control_led']
        ),
    ])
