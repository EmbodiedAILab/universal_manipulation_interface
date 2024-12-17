from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zmq_bridge',
            executable='camera_bridge',
            name='camera_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5566'},
                {'ros_topic': '/zed_camera_image_color'},
                {'zmq_message_name': 'camera_0'}
            ]
        ),

        Node(
            package='zmq_bridge',
            executable='camera_bridge',
            name='camera_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5567'},
                {'ros_topic': '/left_camera_image_color'},
                {'zmq_message_name': 'camera_1'}
            ]
        ),
    ])
