from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Statische Transformation: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Optional: map -> odom (damit der TF-Baum vollständig ist)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # RGB-D Odometry (aus rtabmap_odom!)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[
                {'frame_id': 'base_link'},
                {'subscribe_rgbd': False},
                {'subscribe_rgb': True},
                {'subscribe_depth': True},
                {'approx_sync': True},
                {'queue_size': 30}
            ],
            remappings=[
                ('rgb/image', '/kinect1/rgb'),
                ('depth/image', '/kinect1/depth'),
                ('rgb/camera_info', '/kinect1/camera_info'),
            ]
        ),

        # RTAB-Map Hauptknoten
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'frame_id': 'base_link'},
                {'subscribe_rgbd': False},
                {'subscribe_depth': True},
                {'subscribe_rgb': True},
                {'subscribe_odom_info': True},
                {'queue_size': 30},
                {'Reg/Force3DoF': True},
            ],
            remappings=[
                ('rgb/image', '/kinect1/rgb'),
                ('depth/image', '/kinect1/depth'),
                ('rgb/camera_info', '/kinect1/camera_info'),
                ('odom', '/odom')
            ]
        ),

        # Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[
                {'frame_id': 'base_link'},
                {'subscribe_rgbd': False},
                {'subscribe_rgb': True},
                {'subscribe_depth': True},
            ],
            remappings=[
                ('rgb/image', '/kinect1/rgb'),
                ('depth/image', '/kinect1/depth'),
                ('rgb/camera_info', '/kinect1/camera_info'),
                ('odom', '/odom')
            ]
        ),
    ])

