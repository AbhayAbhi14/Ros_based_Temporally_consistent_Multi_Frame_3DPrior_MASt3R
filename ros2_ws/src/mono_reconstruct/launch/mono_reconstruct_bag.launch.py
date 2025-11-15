import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    use_rosbag = LaunchConfiguration("use_rosbag")

    declare_use_rosbag = DeclareLaunchArgument(
        "use_rosbag",
        default_value="false",
        description="Set true when playing from rosbag"
    )

    # RViz config path
    rviz_config_path = os.path.join(
        os.path.dirname(__file__),
        "mono_recon_viz.rviz"
    )
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    return LaunchDescription([
        declare_use_rosbag,

        # -------------------------------------------------------------
        # USB Camera Node (DISABLED when use_rosbag:=true)
        # -------------------------------------------------------------
        Node(
            condition=UnlessCondition(use_rosbag),
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'frame_rate': 30.0,
            }],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),

        # -------------------------------------------------------------
        # MASt3R Node
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # Temporal Fusion (Window Based)
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # Fixed Window Fusion
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # TF Broadcaster
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # RViz Visualization
        # -------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        ),
    ])
