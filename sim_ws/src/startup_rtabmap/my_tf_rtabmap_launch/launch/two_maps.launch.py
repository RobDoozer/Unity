from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def load_tf_nodes(yaml_path):
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
    # === Static TFs laden ===
    tf_nodes_1 = load_tf_nodes(os.path.join(os.path.dirname(__file__), '..', 'config', 'tf_transforms.yaml'))
    tf_nodes_2 = load_tf_nodes(os.path.join(os.path.dirname(__file__), '..', 'config', 'tf_transforms2.yaml'))

    # === RTABMAP 1 ===
    rtabmap1 = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap1',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            'rgb_topic': '/kinect2/qhd/image_color_rect',
            'depth_topic': '/kinect2/qhd/image_depth_rect',
            'camera_info_topic': '/kinect2/qhd/camera_info',
            'approx_sync': True,
            'rgbd_sync': True,
            'Reg/Strategy': 1,
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'database_path': os.path.expanduser('~/.ros/rtabmap1.db'),
            'subscribe_depth': True,
            'subscribe_scan': True,
            'Grid/FromDepth': True,
            'Grid/RangeMax': 15.0,
            'use_sim_time': False,
            'publish_tf_map': True,
            'publish_tf_odom': True
        }]
    )

    # === RTABMAP 2 ===
    rtabmap2 = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap2',
        output='screen',
        parameters=[{
            'frame_id': 'base_link2',
            'odom_frame_id': 'odom2',
            'scan_topic': '/scan2',
            'rgb_topic': '/kinect1/qhd/image_color_rect',
            'depth_topic': '/kinect1/qhd/image_depth_rect',
            'camera_info_topic': '/kinect1/qhd/camera_info',
            'approx_sync': True,
            'rgbd_sync': True,
            'Reg/Strategy': 1,
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'database_path': os.path.expanduser('~/.ros/rtabmap2.db'),
            'subscribe_depth': True,
            'subscribe_scan': True,
            'Grid/FromDepth': True,
            'Grid/RangeMax': 6.0,
            'use_sim_time': False,
            'publish_tf_map': True,
            'publish_tf_odom': True
        }],
        remappings=[
            ('/map', '/map2'),
            ('/map_graph', '/map_graph2'),
            ('/map_data', '/map_data2')
        ]
    )

    # === RViz (optional) ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription(
        tf_nodes_1 + tf_nodes_2 +
        [rtabmap1, rtabmap2, rviz_node]
    )

