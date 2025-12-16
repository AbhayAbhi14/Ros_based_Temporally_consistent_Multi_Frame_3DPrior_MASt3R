import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_external_camera():
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True,
            text=True
        )

        lines = result.stdout.splitlines()
        devices = {}
        current_name = None

        for line in lines:
            if line.strip().endswith(':'):
                current_name = line.strip().lower()
            elif '/dev/video' in line and current_name:
                devices[line.strip()] = current_name

       
        for dev, name in devices.items():
            if 'logitech' in name or 'brio' in name:
                print(f"[INFO] Using Logitech camera: {dev} ({name})")
                return dev

        
        for dev, name in devices.items():
            if 'usb' in name and 'uvc' not in name:
                print(f"[INFO] Using external USB camera: {dev} ({name})")
                return dev

        
        print("No external camera found, falling back to /dev/video0")
        return '/dev/video0'

    except Exception as e:
        print(f"[ERROR] Camera detection failed: {e}")
        return '/dev/video0'


def generate_launch_description():

    selected_cloud = LaunchConfiguration('selected_cloud')

    return LaunchDescription([

        # ─────────────────────────────────────
        # LAUNCH ARGUMENT
        # ─────────────────────────────────────
        DeclareLaunchArgument(
            'selected_cloud',
            default_value='raw',
            description='raw / adaptive / fixed'
        ),

        # ─────────────────────────────────────
        # USB CAMERA (LOGITECH BRIO)
        # Publishes: /camera/image_raw
        # ─────────────────────────────────────
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': find_external_camera(),
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'usb_camera',
                'camera_info_url': 'file:///home/abhay/.ros/camera_info/usb_camera.yaml',
                'io_method': 'mmap'
            }]
        ),

        # ─────────────────────────────────────
        # MASt3R NODE
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='mast3r_node',
            name='mast3r_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'publish_pointcloud': True,
                'frame_id': 'camera_link'
            }]
        ),

        # ─────────────────────────────────────
        # TEMPORAL FUSION (ADAPTIVE)
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
        # TEMPORAL FUSION (FIXED)
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
        # CAMERA TF BROADCASTER
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
        # OVERLAY PROJECTOR
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

                # Calibration (update if needed)
                'fx': 615.0,
                'fy': 615.0,
                'cx': 320.0,
                'cy': 240.0
            }]
        ),

        # ─────────────────────────────────────
        # RVIZ
        # ─────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
