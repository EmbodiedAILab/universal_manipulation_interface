from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='umi_control',
            executable='arm_moveit_controller',
            name='arm_moveit_controller',
            parameters=[{'use_sim_time': True}],
            output="screen",
        ),
    ])