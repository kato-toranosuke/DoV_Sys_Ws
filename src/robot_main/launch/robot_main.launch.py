from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_main',
            # namespace='turtlesim1',
            executable='main',
            # name='sim'
        ),
        Node(
            package='ml',
            # namespace='turtlesim2',
            executable='srv_calc_features',
            # name='sim'
        ),
        Node(
            package='ml',
            # namespace='turtlesim2',
            executable='srv_dov_pred',
            # name='sim'
        ),
        Node(
            package='recording',
            # namespace='turtlesim2',
            executable='srv_recording',
            # name='sim'
        ),
        Node(
            package='recording',
            # namespace='turtlesim2',
            executable='sub_set_mic_param',
            # name='sim'
        ),
    ])
