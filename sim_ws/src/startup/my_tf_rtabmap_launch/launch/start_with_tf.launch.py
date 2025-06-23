from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

def generate_launch_description():
    # YAML-Dateipfad lesen
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'tf_transforms.yaml')
    config_file = os.path.realpath(config_file)

    with open(config_file, 'r') as f:
        tf_data = yaml.safe_load(f)

    # Erzeuge static_transform_publisher-Nodes
    tf_nodes = []
    for tf in tf_data['transforms']:
        translation = tf['translation']
        rotation = tf['rotation']
        parent = tf['parent']
        child = tf['child']
        args = [*map(str, translation), *map(str, rotation), parent, child]

        tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{parent}_to_{child}',
                arguments=args,
                output='screen'
            )
        )

    # RTABMap Launch-Datei einbinden (Pfad zum ROS-Installationsverzeichnis)
    rtabmap_launch_path = os.path.join(
        '/opt/ros/humble/share/rtabmap_launch/launch', 'rtabmap.launch.py')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'stereo': 'false',
            'icp_odometry': 'true',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.3',
            'visual_odometry': 'false',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'rgb_topic': '/kinect2/qhd/image_color_rect',
            'depth_topic': '/kinect2/qhd/image_depth_rect',
            'camera_info_topic': '/kinect2/qhd/camera_info',
            'camera_frame_id': 'kinect2_color_optical_frame',
            'compressed': 'false',
            'rgbd_sync': 'true',
            'approx_sync': 'true',
            'depth_registration': 'false',
            'database_path': os.path.expanduser('~/.ros/rtabmap.db'),
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf_map': 'true',
            'publish_tf_odom': 'true',
            'use_sim_time': 'false',
            'rviz': 'true',
            'Grid/FromDepth': 'true',
            'Grid/RayTracing': 'true',
            'Grid/MaxGroundHeight': '0.1',
            'Grid/DepthDecimation': '1',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '15.0',
            'Grid/NormalK': '15',
            'RGBD/LinearUpdate': '0.03',
            'RGBD/AngularUpdate': '0.005',
            'RGBD/OptimizeMaxError': '2.0',
            'Mem/NotLinkedNodesKept': 'false',
            'VisMinInliers': '10',
            'Reg/Strategy': '1',  # ICP
            'Odom/MaxTranslation': '0.25',
            'Odom/MaxRotation': '0.8'
        }.items()
    )


    return LaunchDescription(tf_nodes + [rtabmap_launch])

