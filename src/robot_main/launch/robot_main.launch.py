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
            package='robot_main',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='main',
            name=['robot', LaunchConfiguration('robot_id'), '_robot_main']
        ),
        Node(
            package='ml',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='srv_calc_features',
            name=['robot', LaunchConfiguration(
                'robot_id'), '_srv_calc_features']
        ),
        Node(
            package='ml',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='srv_dov_pred',
            name=['robot', LaunchConfiguration('robot_id'), '_srv_dov_pred']
        ),
        Node(
            package='recording',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='srv_recording',
            name=['robot', LaunchConfiguration('robot_id'), '_srv_recording']
        ),
        Node(
            package='recording',
            # namespace=['robot', LaunchConfiguration('robot_id')],
            executable='sub_set_mic_param',
            name=['robot', LaunchConfiguration(
                'robot_id'), '_sub_set_mic_param']
        ),
    ])
