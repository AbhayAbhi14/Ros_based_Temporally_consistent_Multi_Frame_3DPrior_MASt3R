# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():

#     selected_cloud = LaunchConfiguration('selected_cloud')

#     return LaunchDescription([

#         # ─────────────────────────────────────
#         # ARGUMENTS
#         # ─────────────────────────────────────
#         DeclareLaunchArgument(
#             'selected_cloud',
#             default_value='raw',
#             description='raw / adaptive / fixed'
#         ),

#         # ─────────────────────────────────────
#         # INTEL REALSENSE CAMERA (RGB ONLY)
#         # ─────────────────────────────────────
#         Node(
#             package='realsense2_camera',
#             executable='realsense2_camera_node',
#             name='realsense_camera',
#             namespace='camera',
#             output='screen',
#             parameters=[{
#                 # ENABLE STREAMS
#                 'enable_color': True,
#                 'enable_depth': False,
#                 'enable_infra1': False,
#                 'enable_infra2': False,
#                 'enable_gyro': False,
#                 'enable_accel': False,

#                 # COLOR STREAM
#                 'rgb_camera.profile': '640x480x30',
#                 'color_format': 'RGB8',

#                 # SYNC & STABILITY
#                 'enable_sync': True,
#                 'initial_reset': True,

#                 # 🔥 CRITICAL: FORCE LIBREALSENSE (NOT V4L2)
#                 'use_v4l2': False,

#                 # TF HANDLING (we publish our own)
#                 'publish_tf': False,

#                 # TIME
#                 'use_system_time': True
#             }]
#         ),

#         # ─────────────────────────────────────
#         # MASt3R NODE (RGB MONOCULAR)
#         # ─────────────────────────────────────
#         Node(
#             package='mono_reconstruct',
#             executable='mast3r_node',
#             name='mast3r_node',
#             output='screen',
#             parameters=[{
#                 'image_topic': '/camera/color/image_raw',
#                 'publish_pointcloud': True,
#                 'frame_id': 'camera_link'
#             }]
#         ),

#         # ─────────────────────────────────────
#         # TEMPORAL FUSION — ADAPTIVE
#         # ─────────────────────────────────────
#         Node(
#             package='mono_reconstruct',
#             executable='temporal_fusion_node',
#             name='temporal_fusion_node',
#             output='screen',
#             parameters=[{
#                 'fusion_window': 5,
#                 'alpha': 0.4,
#                 'input_topic': '/mast3r/pointcloud',
#                 'output_topic': '/mast3r/fused_pointcloud_adaptive',
#                 'frame_id': 'map',
#                 'publish_rate': 5.0
#             }]
#         ),

#         # ─────────────────────────────────────
#         # TEMPORAL FUSION — FIXED
#         # ─────────────────────────────────────
#         Node(
#             package='mono_reconstruct',
#             executable='temporal_fusion_fixed',
#             name='temporal_fusion_fixed',
#             output='screen',
#             parameters=[{
#                 'input_topic': '/mast3r/pointcloud',
#                 'output_topic': '/mast3r/fused_pointcloud_fixed',
#                 'alpha': 0.4,
#                 'icp_distance': 0.05,
#                 'voxel_size': 0.02,
#                 'frame_id': 'map'
#             }]
#         ),

#         # ─────────────────────────────────────
#         # CAMERA → MAP TF BROADCASTER
#         # ─────────────────────────────────────
#         Node(
#             package='mono_reconstruct',
#             executable='camera_tf_broadcaster',
#             name='camera_tf_broadcaster',
#             output='screen',
#             parameters=[{
#                 'world_frame': 'map',
#                 'camera_frame': 'camera_link'
#             }]
#         ),

#         # ─────────────────────────────────────
#         # OVERLAY PROJECTOR (POINTCLOUD → IMAGE)
#         # ─────────────────────────────────────
#         Node(
#             package='mono_reconstruct',
#             executable='overlay_projector_tf',
#             name='overlay_projector_tf',
#             output='screen',
#             parameters=[{
#                 'image_topic': '/camera/color/image_raw',

#                 'raw_cloud_topic': '/mast3r/pointcloud',
#                 'adaptive_cloud_topic': '/mast3r/fused_pointcloud_adaptive',
#                 'fixed_cloud_topic': '/mast3r/fused_pointcloud_fixed',

#                 'selected_cloud': selected_cloud,
#                 'output_image_topic': '/overlay/image',
#                 'camera_frame': 'camera_link',

#                 # Intrinsics (can also subscribe to camera_info later)
#                 'fx': 615.0,
#                 'fy': 615.0,
#                 'cx': 320.0,
#                 'cy': 240.0
#             }]
#         ),

#         # ─────────────────────────────────────
#         # RVIZ
#         # ─────────────────────────────────────
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen'
#         )
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    selected_cloud = LaunchConfiguration('selected_cloud')

    return LaunchDescription([

        # ─────────────────────────────────────
        # ARGUMENTS
        # ─────────────────────────────────────
        DeclareLaunchArgument(
            'selected_cloud',
            default_value='raw',
            description='raw / adaptive / fixed'
        ),

        # ─────────────────────────────────────
        # INTEL REALSENSE CAMERA (LIBREALSENSE)
        # ─────────────────────────────────────
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            namespace='camera',
            output='screen',
            parameters=[{
                # STREAM ENABLE
                'enable_color': True,
                'enable_depth': True,

                # FORCE PROFILE (MATCH REAL DEVICE)
                'rgb_camera.profile': '1280x720x30',
                'depth_module.profile': '640x480x30',

                # ALIGN + SYNC
                'align_depth.enable': True,
                'enable_sync': True,

                # CRITICAL: DO NOT USE V4L2
                'use_v4l2': False,

                # STABILITY
                'initial_reset': True,
                'publish_tf': False
            }]
        ),

        # ─────────────────────────────────────
        # MASt3R NODE (RGB + CAMERA_INFO)
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='mast3r_node',
            name='mast3r_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'publish_pointcloud': True,
                'frame_id': 'camera_color_optical_frame'
            }]
        ),

        # ─────────────────────────────────────
        # TEMPORAL FUSION — ADAPTIVE
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
        # TEMPORAL FUSION — FIXED
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
        # CAMERA → MAP TF BROADCASTER
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='camera_tf_broadcaster',
            name='camera_tf_broadcaster',
            output='screen',
            parameters=[{
                'world_frame': 'map',
                'camera_frame': 'camera_color_optical_frame'
            }]
        ),

        # ─────────────────────────────────────
        # OVERLAY PROJECTOR (AUTO INTRINSICS)
        # ─────────────────────────────────────
        Node(
            package='mono_reconstruct',
            executable='overlay_projector_tf',
            name='overlay_projector_tf',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',

                'raw_cloud_topic': '/mast3r/pointcloud',
                'adaptive_cloud_topic': '/mast3r/fused_pointcloud_adaptive',
                'fixed_cloud_topic': '/mast3r/fused_pointcloud_fixed',

                'selected_cloud': selected_cloud,
                'output_image_topic': '/overlay/image',
                'camera_frame': 'camera_color_optical_frame'
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
