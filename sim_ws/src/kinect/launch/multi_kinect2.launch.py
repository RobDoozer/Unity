from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinect',
            executable='kinect_receiver2',
            name='kinect3',
            parameters=[
                {'kinect_id': 'kinect3'},
                {'depth_port': 5011},
                {'rgb_port': 5012},
            ]
        ),
        Node(
            package='kinect',
            executable='kinect_receiver2',
            name='kinect4',
            parameters=[
                {'kinect_id': 'kinect4'},
                {'depth_port': 5013},
                {'rgb_port': 5014},
            ]
        )
    ])
