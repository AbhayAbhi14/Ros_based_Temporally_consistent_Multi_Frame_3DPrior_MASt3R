import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

        # Prefer external cameras
        for dev, name in devices.items():
            if any(x in name.lower() for x in ['usb', 'external', 'logitech', 'realsense', 'gopro']):
                return dev

        # Fall back to non-integrated camera
        for dev, name in devices.items():
            if not any(x in name.lower() for x in ['integrated', 'internal']):
                return dev

        return list(devices.keys())[0] if devices else '/dev/video0'

    except:
        return '/dev/video0'


def build_nodes(context, *args, **kwargs):

    camera_type = LaunchConfiguration('camera_type').perform(context)
    rtsp_url = LaunchConfiguration('rtsp_url').perform(context)
    selected_cloud = LaunchConfiguration('selected_cloud').perform(context)

    nodes = []

    # ─────────────────────────────────────
    # CAMERA INPUT (USB / IP)
    # ─────────────────────────────────────
    if camera_type == "usb":
        print("Using USB Camera")

        nodes.append(
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
                remappings=[('/image_raw', '/camera/image_raw')]
            )
        )

    elif camera_type == "ip":
        print("Using IP Camera:", rtsp_url)

        nodes.append(
            Node(
                package='ffmpeg_camera',
                executable='ffmpeg_camera_node',
                name='ip_camera',
                output='screen',
                parameters=[{
                    'camera_name': 'ip_cam',
                    'fps': 30.0,
                    'frame_id': 'camera_link',
                    'rtsp_url': rtsp_url,
                    'image_encoding': 'rgb8'
                }],
                remappings=[('/image_raw', '/camera/image_raw')]
            )
        )

    else:
        print("AUTO Camera selection enabled")

        external = find_external_camera()
        if external:
            print("Detected USB camera:", external)
            nodes.append(
                Node(
                    package='v4l2_camera',
                    executable='v4l2_camera_node',
                    name='usb_camera',
                    output='screen',
                    parameters=[{
                        'video_device': external,
                        'image_size': [640, 480],
                        'frame_rate': 30.0
                    }],
                    remappings=[('/image_raw', '/camera/image_raw')]
                )
            )
        else:
            print("No USB camera found → Using IP fallback")
            nodes.append(
                Node(
                    package='ffmpeg_camera',
                    executable='ffmpeg_camera_node',
                    name='ip_camera',
                    output='screen',
                    parameters=[{
                        'camera_name': 'ip_cam',
                        'fps': 30.0,
                        'frame_id': 'camera_link',
                        'rtsp_url': rtsp_url,
                        'image_encoding': 'rgb8'
                    }],
                    remappings=[('/image_raw', '/camera/image_raw')]
                )
            )

    # ─────────────────────────────────────
    # MASt3R Node
    # ─────────────────────────────────────
    nodes.append(
        Node(
            package='mono_reconstruct',
            executable='mast3r_node',
            name='mast3r_node',
            output='screen',
            parameters=[{
                'publish_pointcloud': True,
                'frame_id': 'camera_link'
            }]
        )
    )

    # ─────────────────────────────────────
    # Temporal Fusion Nodes
    # ─────────────────────────────────────
    nodes.extend([
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
                'frame_id': 'map'
            }]
        ),
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
        )
    ])

    # ─────────────────────────────────────
    # TF Broadcaster
    # ─────────────────────────────────────
    nodes.append(
        Node(
            package='mono_reconstruct',
            executable='camera_tf_broadcaster',
            name='camera_tf_broadcaster',
            output='screen',
            parameters=[{
                'world_frame': 'map',
                'camera_frame': 'camera_link'
            }]
        )
    )

    # ─────────────────────────────────────
    # Overlay Projector
    # ─────────────────────────────────────
    nodes.append(
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
                'fx': 420.0,
                'fy': 420.0,
                'cx': 320.0,
                'cy': 240.0
            }]
        )
    )

    # ─────────────────────────────────────
    # Depth Comparison Node (Fixed location)
    # ─────────────────────────────────────
    nodes.append(
        Node(
            package='mono_reconstruct',
            executable='compare_depth',
            name='compare_depth',
            output='screen',
            parameters=[{
                'mast3r_topic': '/mast3r/depth_image',
                'realsense_topic': '/camera/depth/image_rect_raw',
                'output_topic': '/depth_comparison'
            }]
        )
    )

    # ─────────────────────────────────────
    # RViz2
    # ─────────────────────────────────────
    nodes.append(
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'camera_type',
            default_value='auto',
            description='usb / ip / auto'
        ),

        DeclareLaunchArgument(
            'rtsp_url',
            default_value='rtsp://192.168.1.10:554/stream1',
            description='RTSP IP camera URL'
        ),

        DeclareLaunchArgument(
            'selected_cloud',
            default_value='raw',
            description='raw / adaptive / fixed'
        ),

        OpaqueFunction(function=build_nodes)
    ])
