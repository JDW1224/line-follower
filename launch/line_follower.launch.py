from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Camera Publisher Node (Python - v4l2)
    camera_publisher_node = Node(
        package='line_follower',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen'
    )
    
    line_follower_node = Node(
        package='line_follower',
        executable='line_follower',
        name='line_follower',
        output='screen'
    )

    camera_viewer_node = Node(
        package='line_follower',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen'
    )

    # lane_detector_node = Node(
    #     package='line_follower',
    #     executable='lane_detector.py',     # ← 추가
    #     name='lane_detector',
    #     output='screen'
    # )

    return LaunchDescription([
        camera_publisher_node,
        line_follower_node,
        camera_viewer_node,
        # lane_detector_node,
    ])