import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def find_external_camera():
    """
    Detects an external USB camera automatically by parsing `v4l2-ctl --list-devices`.
    Prefers external cameras (USB, GoPro, Logitech, etc.).
    """
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        lines = result.stdout.splitlines()
        devices = {}
        current_name = None

        for line in lines:
            if ':' in line:
                current_name = line.strip().split(':')[0]
            elif '/dev/video' in line and current_name:
                devices[line.strip()] = current_name

        # Prefer external cameras
        for dev, name in devices.items():
            if any(x in name.lower() for x in ['usb', 'external', 'logitech', 'realsense', 'gopro']):
                return dev

        # Fallback: avoid integrated webcams
        for dev, name in devices.items():
            if not any(x in name.lower() for x in ['integrated', 'laptop webcam', 'hd webcam']):
                return dev

        # Final fallback
        return list(devices.keys())[0] if devices else '/dev/video0'

    except Exception:
        return '/dev/video0'


def generate_launch_description():
    # Auto-select camera
    camera_device = find_external_camera()
    print(f"📸 Auto-selected camera device: {camera_device}")

    # RViz configuration file path
    rviz_config_path = os.path.join(
        os.path.dirname(__file__),
        'mono_recon_viz.rviz'
    )
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    return LaunchDescription([

        # ────────────────────────────────
        # USB Camera Node
        # ────────────────────────────────
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'video_device': camera_device,
                'image_size': [640, 480],
                'frame_rate': 30.0
            }],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),

        # ────────────────────────────────
        # MASt3R Reconstruction Node
        # ────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='mast3r_node',
            name='mast3r_node',
            output='screen',
            parameters=[{
                'publish_pointcloud': True,
                'frame_id': 'camera_link'
            }],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/mast3r/pointcloud', '/mast3r/pointcloud'),
                ('/mast3r/depth', '/mast3r/depth')
            ]
        ),

        # ────────────────────────────────
        # Temporal Fusion Node (Adaptive)
        # ────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='temporal_fusion_node',
            name='temporal_fusion_node',
            output='screen',
            parameters=[{
                'fusion_window': 5,
                'alpha': 0.4,
                'input_topic': '/mast3r/pointcloud',
                'output_topic': '/mast3r/fused_pointcloud_adaptive',
                'frame_id': 'map',
                'publish_rate': 5.0
            }],
            remappings=[
                ('/mast3r/pointcloud', '/mast3r/pointcloud'),
                ('/mast3r/fused_pointcloud', '/mast3r/fused_pointcloud_adaptive')
            ]
        ),

        # ────────────────────────────────
        #  Fixed Window Fusion Node (New)
        # ────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='temporal_fusion_fixed',
            name='temporal_fusion_fixed',
            output='screen',
            parameters=[{
                'input_topic': '/mast3r/pointcloud',
                'output_topic': '/mast3r/fused_pointcloud_fixed',
                'alpha': 0.4,
                'icp_distance': 0.05,
                'voxel_size': 0.02,
                'frame_id': 'map'
            }],
            remappings=[
                ('/mast3r/pointcloud', '/mast3r/pointcloud'),
                ('/mast3r/fused_pointcloud', '/mast3r/fused_pointcloud_fixed')
            ]
        ),

        # ────────────────────────────────
        #  TF Broadcaster
        # ────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='camera_tf_broadcaster',
            name='camera_tf_broadcaster',
            output='screen',
            parameters=[{
                'world_frame': 'map',
                'camera_frame': 'camera_link'
            }]
        ),

        # ────────────────────────────────
        #  RViz2 Visualization
        # ────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        ),
    ])
