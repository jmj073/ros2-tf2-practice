import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('learning_tf2_cpp'), 'launch'),
            '/turtle_tf2_demo.launch.py',
        ]),
        launch_arguments={
            'turtle2_target_frame': 'carrot1',
            'turtle3_target_frame': 'carrot2',
        }.items(),
    )

    return LaunchDescription([
        demo_nodes,
        Node(
            package='learning_tf2_cpp',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
        Node(
            package='learning_tf2_cpp',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
    ])
