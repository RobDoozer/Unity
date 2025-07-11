from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinect',
            executable='kinect_receiver',
            name='kinect2',
            parameters=[
                {'kinect_id': 'kinect2'},
                {'depth_port': 5009},
                {'rgb_port': 5010},
            ]
        )
    ])
