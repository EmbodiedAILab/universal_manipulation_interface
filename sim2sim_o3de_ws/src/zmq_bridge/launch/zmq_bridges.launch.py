from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ================= Observations =================
        # cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('zmq_bridge'),
                    'launch/multi_cameras.launch.py'
                ])
            ),
        ),

        # robot states
        Node(
            package='zmq_bridge',
            executable='states_bridge',
            name='states_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5555'},
                {'eef_pose_topic': '/eef_pose'},
                {'gripper_width_topic': '/gripper_width'},
                {'vacuum_status_topic': '/vacuum_status'}
            ]
        ),

        # ================= Actions =================
        # arm control
        Node(
            package='zmq_bridge',
            executable='arm_control_bridge',
            name='arm_control_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5554'},
                {'servoL_topic_name': '/eef_servo_controller'},
                {'moveJ_topic_name': '/moveJ_cmd'}
            ]
        ),

        # gripper control
        Node(
            package='zmq_bridge',
            executable='gripper_control_bridge',
            name='gripper_control_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5556'},
                {'gripper_control_topic_name': '/gripper_cmd'}
            ]
        ),

        # vacuum control
        Node(
            package='zmq_bridge',
            executable='vacuum_control_bridge',
            name='vacuum_control_bridge',
            parameters=[
                {'zmq_host': '127.0.0.1'},
                {'zmq_port': '5557'},
                {'vacuum_control_topic_name': '/vacuum_cmd'}
            ]
        ),
    ])
