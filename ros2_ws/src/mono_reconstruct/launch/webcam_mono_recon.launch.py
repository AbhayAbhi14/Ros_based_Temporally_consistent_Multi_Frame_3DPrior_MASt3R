import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_external_camera():
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

        for dev, name in devices.items():
            if any(x in name.lower() for x in ['usb', 'external', 'logitech', 'realsense', 'gopro']):
                return dev

        for dev, name in devices.items():
            if not any(x in name.lower() for x in ['integrated', 'laptop webcam', 'hd webcam']):
                return dev

        return list(devices.keys())[0] if devices else '/dev/video0'

    except Exception:
        return '/dev/video0'


def generate_launch_description():

    selected_cloud = LaunchConfiguration('selected_cloud')

    return LaunchDescription([

        # ─────────────────────────────────────
        # Launch ARGUMENTS
        # ─────────────────────────────────────
        DeclareLaunchArgument(
            'selected_cloud',
            default_value='raw',
            description='Choose which cloud to overlay: raw / adaptive / fixed'
        ),

        # ─────────────────────────────────────
        # Camera Node
        # ─────────────────────────────────────
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'video_device': find_external_camera(),
                'image_size': [640, 480],
                'frame_rate': 30.0
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ]
        ),

        # ─────────────────────────────────────
        # MASt3R Node
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='mast3r_node',
            name='mast3r_node',
            output='screen',
            parameters=[{
                'publish_pointcloud': True,
                'frame_id': 'camera_link'
            }]
        ),

        # ─────────────────────────────────────
        # Temporal Fusion (Adaptive)
        # ─────────────────────────────────────
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
            }]
        ),

        # ─────────────────────────────────────
        # Fixed Window Fusion
        # ─────────────────────────────────────
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
            }]
        ),

        # ─────────────────────────────────────
        # TF Broadcaster
        # ─────────────────────────────────────
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

        # ─────────────────────────────────────
        # Overlay Projector (Your Node)
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='overlay_projector_tf',
            name='overlay_projector_tf',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'raw_cloud_topic': '/mast3r/pointcloud',
                'adaptive_cloud_topic': '/mast3r/fused_pointcloud_adaptive',
                'fixed_cloud_topic': '/mast3r/fused_pointcloud_fixed',
                'selected_cloud': selected_cloud,
                'output_image_topic': '/overlay/image',
                'camera_frame': 'camera_link',

                # Camera intrinsics
                'fx': 420.0,
                'fy': 420.0,
                'cx': 320.0,
                'cy': 240.0
            }]
        ),

        # ─────────────────────────────────────
        # RViz2
        # ─────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
