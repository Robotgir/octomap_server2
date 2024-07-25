from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server2',
            executable='octomap_saver_node',
            name='octomap_saver',
            output='screen',
            parameters=[
                {'full': True},  # Set to True or False depending on the type of octomap you want
                {'octomap_path': 'octomap.bt'},  # Replace with your desired save path
            ],
        ),
    ])
