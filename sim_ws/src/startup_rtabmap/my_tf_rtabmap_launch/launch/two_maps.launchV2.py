from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

def load_tf_nodes(yaml_path):
    """Lade statische TF-Nodes aus YAML-Datei"""
    with open(yaml_path, 'r') as f:
        tf_data = yaml.safe_load(f)

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
    return tf_nodes

def generate_launch_description():
    # Pfad zur RTAB-Map Launch-Datei
    rtabmap_launch_path = os.path.join(
        '/opt/ros/humble/share/rtabmap_launch/launch', 'rtabmap.launch.py'
    )

    # --- RTAB-Map 1: Kinect2 + /scan ---
    tf_nodes_1 = load_tf_nodes(os.path.realpath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'tf_transforms.yaml')
    ))

    rtabmap1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'stereo': 'false',
            'icp_odometry': 'true',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.3',
            'visual_odometry': 'false',
            'subscribe_scan': 'true',
            'icp_odometry': 'true',
            'scan_topic': '/scan',
            'rgb_topic': '/kinect2/qhd/image_color_rect',
            'depth_topic': '/kinect2/qhd/image_depth_rect',
            'camera_info_topic': '/kinect2/qhd/camera_info',
            'camera_frame_id': 'kinect2_color_optical_frame',
            'compressed': 'false',
            'rgbd_sync': 'true',
            'depth_registration': 'false',
            'database_path': os.path.expanduser('~/.ros/rtabmap1.db'),
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
            'Reg/Strategy': '1',
            'Odom/MaxTranslation': '0.25',
            'Odom/MaxRotation': '0.8'
        }.items()
    )

    # --- RTAB-Map 2: Kinect1 + /scan2 ---
    tf_nodes_2 = load_tf_nodes(os.path.realpath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'tf_transforms2.yaml')
    ))

    rtabmap2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'stereo': 'false',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.3',
            'visual_odometry': 'false',
            'subscribe_scan': 'true',
            'scan_topic': '/scan2',
            'rgb_topic': '/kinect1/qhd/image_color_rect',
            'depth_topic': '/kinect1/qhd/image_depth_rect',
            'camera_info_topic': '/kinect1/qhd/camera_info',
            'camera_frame_id': 'kinect1_color_optical_frame',
            'compressed': 'false',
            'rgbd_sync': 'true',
            'depth_registration': 'false',
            'database_path': os.path.expanduser('~/.ros/rtabmap2.db'),
            'frame_id': 'base_link2',
            'odom_frame_id': 'odom2',
            'publish_tf_map': 'true',
            'publish_tf_odom': 'true',
            'use_sim_time': 'false',
            'rviz': 'false',
            'Grid/FromDepth': 'true',
            'Grid/RayTracing': 'true',
            'Grid/MaxGroundHeight': '0.1',
            'Grid/DepthDecimation': '1',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '6.0',
            'Grid/NormalK': '15',
            'RGBD/LinearUpdate': '0.03',
            'RGBD/AngularUpdate': '0.005',
            'RGBD/OptimizeMaxError': '2.0',
            'Mem/NotLinkedNodesKept': 'false',
            'VisMinInliers': '10',
            'Odom/ResetCountdown': '1',
            'Odom/Strategy': '1',
            'Odom/GuessMotion': 'true',
            'RGBD/OptimizeFromGraphEnd': 'true',
            'Mem/RehearsalSimilarity': '0.3',
            'Rtabmap/TimeThr': '700',
            'subscribe_odom': 'true',
            'RGBD/NeighborLinkRefining': 'true'
        }.items()
    )

    # Rückgabe aller Nodes und beiden RTABMap-Instanzen
    return LaunchDescription(tf_nodes_1 + tf_nodes_2 + [rtabmap1, rtabmap2])

