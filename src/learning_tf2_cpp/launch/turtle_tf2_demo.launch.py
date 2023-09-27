from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node( # simulator
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        ############## turtle1 ##############
        Node( # tf2 broadcaster for turtle1
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        ############## turtle2 ##############
        DeclareLaunchArgument(
            'turtle2_target_frame', default_value='turtle1',
            description='Target frame name for turtle2.'
        ),
        Node( # tf2 broadcaster for turtle2
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node( # tf2 listener src: source_frame(turtle2), targ(turtle1)
            package='learning_tf2_cpp',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('turtle2_target_frame')},
                {'source_frame': 'turtle2'},
                {'forward_speed': 0.7},
                {'rotation_rate': 1.0},
            ]
        ),

        ############## turtle3 ##############
        DeclareLaunchArgument(
            'turtle3_target_frame', default_value='turtle2',
            description='Target frame name for turtle3.'
        ),
        Node( # tf2 broadcaster for turtle3
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster3',
            parameters=[
                {'turtlename': 'turtle3'},
            ]
        ),
        Node( # tf2 listener src: source_frame(turtle3), targ(turtle2)
            package='learning_tf2_cpp',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('turtle3_target_frame')},
                {'source_frame': 'turtle3'},
                {'forward_speed': 1.0},
                {'rotation_rate': 1.5},
            ]
        ),
    ])
