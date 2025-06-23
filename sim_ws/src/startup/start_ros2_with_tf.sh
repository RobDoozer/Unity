#!/bin/bash

# Pfad zur YAML-Datei
TF_CONFIG="tf_transforms.yaml"

# Lies YAML und starte static_transform_publisher
python3 - <<EOF
import yaml
import subprocess

with open("$TF_CONFIG", 'r') as f:
    data = yaml.safe_load(f)

for tf in data['transforms']:
    trans = ' '.join(map(str, tf['translation']))
    rot = ' '.join(map(str, tf['rotation']))
    parent = tf['parent']
    child = tf['child']
    cmd = f"ros2 run tf2_ros static_transform_publisher {trans} {rot} {parent} {child}"
    subprocess.Popen(cmd, shell=True)
EOF

# Kurze Pause, um sicherzustellen, dass alle TFs gestartet sind
sleep 3

# Starte RTAB-Map
stdbuf -o L ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=false \
    icp_odometry:=false \
    visual_odometry:=false \
    subscribe_scan:=true \
    scan_topic:=/scan \
    rgb_topic:=/kinect2/qhd/image_color_rect \
    depth_topic:=/kinect2/qhd/image_depth_rect \
    camera_info_topic:=/kinect2/qhd/camera_info \
    camera_frame_id:=kinect2_color_optical_frame \
    compressed:=false \
    rgbd_sync:=true \
    approx_sync:=true \
    depth_registration:=false \
    database_path:=~/.ros/rtabmap.db \
    frame_id:=base_link \
    odom_frame_id:=odom \
    publish_tf_map:=true \
    publish_tf_odom:=true \
    use_sim_time:=false \
    rviz:=true

