from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pc_main',
            executable='pub_start_rec',
        ),
        Node(
            package='pc_main',
            executable='sub_election',
        ),
    ])
